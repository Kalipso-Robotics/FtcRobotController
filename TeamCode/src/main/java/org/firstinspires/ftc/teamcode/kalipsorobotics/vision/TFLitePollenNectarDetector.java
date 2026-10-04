package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import android.content.Context;
import android.content.res.AssetFileDescriptor;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.tensorflow.lite.DataType;
import org.tensorflow.lite.Interpreter;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * YOLO26 INT8 detector for pollen and nectar.
 *
 * Differs from TFLiteArtifactDetector:
 *   - INT8 input, quantized with the tensor's own scale and zero point.
 *   - Three classes, resolved by argmax rather than a hardcoded label.
 *   - Letterbox instead of stretch, matching how the model was trained.
 *   - Works with square (256x256) and rectangular (e.g. 224x384) inputs. Width and
 *     height are read from the model file, so only MODEL_FILE needs to change.
 *   - Per-class NMS. The LiteRT export ships the dense one-to-many head, which
 *     proposes several overlapping boxes per ball, so duplicates are suppressed here.
 *
 * DIAGNOSTICS: getDiagnosticSummary() reports the model's input/output shape and
 * the strongest class score seen in the last frame, even below the threshold.
 */
public class TFLitePollenNectarDetector extends KVisionProcessor<List<VisionRecognition>> {

    private static final String MODEL_FILE = "deploy384_224x384.tflite";
    private static final String[] LABELS = {"nectar_blue", "nectar_red", "pollen"};

    private static final float DEFAULT_MIN_CONFIDENCE = 0.25f;
    private static final Scalar PAD_COLOR = new Scalar(114, 114, 114);

    private final Context appContext;
    private float minConfidence = DEFAULT_MIN_CONFIDENCE;
    private float nmsIou = 0.5f;

    /** Set true if red and blue come out swapped. */
    private boolean swapRedBlue = false;

    private Interpreter interpreter;

    private int modelInputWidth;
    private int modelInputHeight;
    private int numClasses;
    private int numAnchors;
    private int numChannels;

    private boolean inputIsChannelsFirst;
    private boolean inputIsFloat;
    private boolean outputIsFloat;
    private float inScale = 1f / 255f;
    private int inZeroPoint = -128;
    private float outScale = 1f;
    private int outZeroPoint = 0;

    private ByteBuffer inputBuffer;
    private ByteBuffer outputBuffer;
    private byte[] rawPixelBuffer;
    private float[][] decoded;

    private Mat rgbFrame;
    private Mat scaledFrame;
    private Mat letterboxedFrame;

    private float letterboxScale = 1f;
    private int padX = 0;
    private int padY = 0;

    private Paint boxPaint;
    private Paint labelPaint;

    private List<VisionRecognition> currentFrameRecognitions = new ArrayList<>();

    // Diagnostics: strongest score in the last frame, even below the threshold
    private volatile float lastMaxScore = 0f;
    // Diagnostics: which class index had that score
    private volatile int lastMaxClass = -1;
    // Diagnostics: size and channel count of the incoming camera frame
    private volatile String lastFrameInfo = "none yet";

    public TFLitePollenNectarDetector(Context appContext) {
        this.appContext = appContext;
    }

    @Override
    protected void onInit(int frameWidth, int frameHeight) {
        try {
            // Uses 4 CPU threads, matching the 152 ms benchmark
            Interpreter.Options options = new Interpreter.Options();
            options.setNumThreads(4);
            interpreter = new Interpreter(loadModelFromAssets(appContext, MODEL_FILE), options);
        } catch (IOException exception) {
            throw new RuntimeException("Failed to load TFLite model: " + MODEL_FILE, exception);
        }

        // Newer exports are channels-first [1, 3, H, W]; older ones are channels-last [1, H, W, 3]
        int[] inputShape = interpreter.getInputTensor(0).shape();
        inputIsChannelsFirst = inputShape[1] == 3;
        // Channels-first: [1, 3, H, W]. Channels-last: [1, H, W, 3].
        modelInputHeight = inputIsChannelsFirst ? inputShape[2] : inputShape[1];
        modelInputWidth = inputIsChannelsFirst ? inputShape[3] : inputShape[2];
        inputIsFloat = interpreter.getInputTensor(0).dataType() == DataType.FLOAT32;

        int[] outputShape = interpreter.getOutputTensor(0).shape();
        numChannels = outputShape[1];
        numAnchors = outputShape[2];
        numClasses = numChannels - 4;
        outputIsFloat = interpreter.getOutputTensor(0).dataType() == DataType.FLOAT32;

        inScale = interpreter.getInputTensor(0).quantizationParams().getScale();
        inZeroPoint = interpreter.getInputTensor(0).quantizationParams().getZeroPoint();
        if (inScale == 0f) inScale = 1f / 255f;

        outScale = interpreter.getOutputTensor(0).quantizationParams().getScale();
        outZeroPoint = interpreter.getOutputTensor(0).quantizationParams().getZeroPoint();
        if (outScale == 0f) outScale = 1f;

        inputBuffer = ByteBuffer.allocateDirect(interpreter.getInputTensor(0).numBytes());
        inputBuffer.order(ByteOrder.nativeOrder());
        outputBuffer = ByteBuffer.allocateDirect(interpreter.getOutputTensor(0).numBytes());
        outputBuffer.order(ByteOrder.nativeOrder());

        rawPixelBuffer = new byte[modelInputWidth * modelInputHeight * 3];
        decoded = new float[numChannels][numAnchors];

        rgbFrame = new Mat();
        scaledFrame = new Mat();
        letterboxedFrame = new Mat();

        boxPaint = makeBoxPaint();
        labelPaint = makeLabelPaint();
    }

    @Override
    protected List<VisionRecognition> detect(Mat frame) {
        lastFrameInfo = frame.width() + "x" + frame.height() + " ch=" + frame.channels();

        Mat source;
        if (frame.channels() == 4) {
            Imgproc.cvtColor(frame, rgbFrame, Imgproc.COLOR_RGBA2RGB);
            source = rgbFrame;
        } else {
            source = frame;
        }

        // Letterbox: scale to fit, pad the short side with 114 grey, as
        // Ultralytics does during training.
        int sourceWidth = source.width();
        int sourceHeight = source.height();
        // 1280x720 into 384 wide x 224 tall: scale 0.3 -> 384x216, 4 px grey top and bottom.
        letterboxScale = Math.min((float) modelInputWidth / sourceWidth,
                (float) modelInputHeight / sourceHeight);
        int scaledWidth = Math.min(modelInputWidth, Math.round(sourceWidth * letterboxScale));
        int scaledHeight = Math.min(modelInputHeight, Math.round(sourceHeight * letterboxScale));
        padX = (modelInputWidth - scaledWidth) / 2;
        padY = (modelInputHeight - scaledHeight) / 2;

        Imgproc.resize(source, scaledFrame, new Size(scaledWidth, scaledHeight));
        Core.copyMakeBorder(scaledFrame, letterboxedFrame,
                padY, modelInputHeight - scaledHeight - padY,
                padX, modelInputWidth - scaledWidth - padX,
                Core.BORDER_CONSTANT, PAD_COLOR);

        letterboxedFrame.get(0, 0, rawPixelBuffer);

        // rawPixelBuffer is interleaved RGBRGB... (channels-last).
        // Channels-first models want all R, then all G, then all B.
        int pixels = modelInputWidth * modelInputHeight;
        inputBuffer.rewind();
        if (inputIsChannelsFirst) {
            for (int c = 0; c < 3; c++) {
                for (int p = 0; p < pixels; p++) {
                    writeInputValue(rawPixelBuffer[p * 3 + c] & 0xFF);
                }
            }
        } else {
            for (int i = 0; i < pixels * 3; i++) {
                writeInputValue(rawPixelBuffer[i] & 0xFF);
            }
        }

        inputBuffer.rewind();
        outputBuffer.rewind();
        interpreter.run(inputBuffer, outputBuffer);
        outputBuffer.rewind();

        for (int c = 0; c < numChannels; c++) {
            for (int a = 0; a < numAnchors; a++) {
                decoded[c][a] = outputIsFloat
                        ? outputBuffer.getFloat()
                        : (outputBuffer.get() - outZeroPoint) * outScale;
            }
        }

        List<VisionRecognition> recognitions = new ArrayList<>();

        float frameMaxScore = 0f;
        int frameMaxClass = -1;

        for (int a = 0; a < numAnchors; a++) {
            int bestClass = -1;
            float bestScore = 0f;
            for (int c = 0; c < numClasses; c++) {
                float score = decoded[4 + c][a];
                if (score > bestScore) {
                    bestScore = score;
                    bestClass = c;
                }
            }

            if (bestScore > frameMaxScore) {
                frameMaxScore = bestScore;
                frameMaxClass = bestClass;
            }

            if (bestClass < 0 || bestScore < minConfidence) continue;

            float cx = decoded[0][a];
            float cy = decoded[1][a];
            float w = decoded[2][a];
            float h = decoded[3][a];

            // Ultralytics normally exports coords normalized 0-1 (x by width, y by height),
            // but some exports emit input pixels. Detect and normalize per axis.
            if (cx > 1.5f || cy > 1.5f) {
                cx /= modelInputWidth;
                cy /= modelInputHeight;
                w /= modelInputWidth;
                h /= modelInputHeight;
            }

            float centerXPx = cx * modelInputWidth;
            float centerYPx = cy * modelInputHeight;
            float widthPx = w * modelInputWidth;
            float heightPx = h * modelInputHeight;

            float left = (centerXPx - widthPx / 2f - padX) / letterboxScale;
            float top = (centerYPx - heightPx / 2f - padY) / letterboxScale;
            float right = (centerXPx + widthPx / 2f - padX) / letterboxScale;
            float bottom = (centerYPx + heightPx / 2f - padY) / letterboxScale;

            // Falls back to "class32" etc. instead of crashing if the model has more than 3 classes
            String label = bestClass < LABELS.length ? LABELS[bestClass] : "class" + bestClass;
            if (swapRedBlue) {
                if (label.equals("nectar_red")) label = "nectar_blue";
                else if (label.equals("nectar_blue")) label = "nectar_red";
            }

            recognitions.add(new VisionRecognition(label, bestScore, left, top, right, bottom));
        }

        lastMaxScore = frameMaxScore;
        lastMaxClass = frameMaxClass;

        recognitions.sort((x, y) -> Float.compare(y.confidence, x.confidence));

        // The exported model uses the dense (one-to-many) head, which proposes
        // several overlapping boxes per ball. Keep the most confident box of each
        // cluster and drop same-class boxes that overlap it heavily.
        List<VisionRecognition> kept = applyNms(recognitions, nmsIou);

        currentFrameRecognitions = kept;
        SharedData.setPollenNectarDetections(kept);
        return kept;
    }

    @Override
    protected void annotate(Canvas canvas, List<VisionRecognition> result, DrawContext drawContext) {
        boxPaint.setStrokeWidth(STROKE_WIDTH_DP * drawContext.screenDensityScale);
        labelPaint.setTextSize(TEXT_SIZE_DP * drawContext.screenDensityScale);

        for (VisionRecognition r : result) {
            int color = colorFor(r.label);
            boxPaint.setColor(color);
            labelPaint.setColor(color);
            canvas.drawRect(
                    r.left * drawContext.bitmapToCanvasScale,
                    r.top * drawContext.bitmapToCanvasScale,
                    r.right * drawContext.bitmapToCanvasScale,
                    r.bottom * drawContext.bitmapToCanvasScale,
                    boxPaint);
            canvas.drawText(r.formattedLabel,
                    r.left * drawContext.bitmapToCanvasScale,
                    r.top * drawContext.bitmapToCanvasScale
                            - LABEL_OFFSET_DP * drawContext.screenDensityScale,
                    labelPaint);
        }
    }

    @Override
    public String getDiagnosticSummary() {
        return String.format(Locale.US,
                "%s | frame=%s | in=%dx%d %s %s | out=%dx%d %s | classes=%d | max=%.3f (class %d)",
                MODEL_FILE, lastFrameInfo,
                modelInputHeight, modelInputWidth, inputIsChannelsFirst ? "CHW" : "HWC", inputIsFloat ? "float" : "int8",
                numChannels, numAnchors, outputIsFloat ? "float" : "int8",
                numClasses, lastMaxScore, lastMaxClass);
    }

    /**
     * Greedy per-class non-maximum suppression. Input must be sorted by confidence,
     * highest first. A box is dropped if it overlaps an already-kept box of the same
     * class by more than iouThreshold.
     */
    private static List<VisionRecognition> applyNms(List<VisionRecognition> sorted, float iouThreshold) {
        List<VisionRecognition> kept = new ArrayList<>();
        for (VisionRecognition candidate : sorted) {
            boolean suppressed = false;
            for (VisionRecognition k : kept) {
                if (k.label.equals(candidate.label) && iou(k, candidate) > iouThreshold) {
                    suppressed = true;
                    break;
                }
            }
            if (!suppressed) kept.add(candidate);
        }
        return kept;
    }

    /** Intersection-over-union of two boxes, 0 (no overlap) to 1 (identical). */
    private static float iou(VisionRecognition a, VisionRecognition b) {
        float interLeft = Math.max(a.left, b.left);
        float interTop = Math.max(a.top, b.top);
        float interRight = Math.min(a.right, b.right);
        float interBottom = Math.min(a.bottom, b.bottom);
        float interArea = Math.max(0f, interRight - interLeft) * Math.max(0f, interBottom - interTop);
        float areaA = (a.right - a.left) * (a.bottom - a.top);
        float areaB = (b.right - b.left) * (b.bottom - b.top);
        float union = areaA + areaB - interArea;
        return union <= 0f ? 0f : interArea / union;
    }

    /** Writes one 0-255 pixel value in whatever number format the model's input expects. */
    private void writeInputValue(int pixel) {
        float normalized = pixel / 255.0f;
        if (inputIsFloat) {
            inputBuffer.putFloat(normalized);
        } else {
            int quantized = Math.round(normalized / inScale) + inZeroPoint;
            inputBuffer.put((byte) Math.max(-128, Math.min(127, quantized)));
        }
    }

    private static int colorFor(String label) {
        if (label.equals("nectar_blue")) return Color.CYAN;
        if (label.equals("nectar_red")) return Color.RED;
        return Color.YELLOW;
    }

    public TFLitePollenNectarDetector withMinConfidence(float confidence) {
        this.minConfidence = confidence;
        return this;
    }

    /**
     * Overlap above which a same-class box counts as a duplicate. Lower = merges more
     * aggressively (can merge touching balls); higher = keeps more boxes (duplicates may return).
     */
    public TFLitePollenNectarDetector withNmsIou(float iouThreshold) {
        this.nmsIou = iouThreshold;
        return this;
    }

    public TFLitePollenNectarDetector withSwappedRedBlue(boolean swap) {
        this.swapRedBlue = swap;
        return this;
    }

    private static MappedByteBuffer loadModelFromAssets(Context context, String modelFileName)
            throws IOException {
        try (AssetFileDescriptor fd = context.getAssets().openFd(modelFileName);
             FileInputStream in = new FileInputStream(fd.getFileDescriptor())) {
            return in.getChannel().map(FileChannel.MapMode.READ_ONLY,
                    fd.getStartOffset(), fd.getDeclaredLength());
        }
    }

    private static Paint makeBoxPaint() {
        Paint paint = new Paint();
        paint.setStyle(Paint.Style.STROKE);
        return paint;
    }

    private static Paint makeLabelPaint() {
        Paint paint = new Paint();
        paint.setStyle(Paint.Style.FILL);
        paint.setTypeface(Typeface.MONOSPACE);
        return paint;
    }
}