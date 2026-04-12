package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import android.content.Context;
import android.content.res.AssetFileDescriptor;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.tensorflow.lite.Interpreter;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Abstract base class for ML-based object detectors trained with Roboflow.
 *
 * HOW TO USE WITH ROBOFLOW:
 *   1. Annotate your images at roboflow.com and export as "TFLite" format.
 *   2. Place the .tflite file in TeamCode/src/main/assets/
 *   3. Create a subclass and implement interpret():
 *
 *   public class YellowRectDetector extends KRoboflowDetector<List<YellowRect>> {
 *
 *       public YellowRectDetector(Context appContext) {
 *           super(appContext, "yellow_rect.tflite", "yellow_rect");
 *       }
 *
 *       @Override
 *       protected List<YellowRect> interpret(List<RoboflowRecognition> recognitions) {
 *           List<YellowRect> results = new ArrayList<>();
 *           for (RoboflowRecognition recognition : recognitions) {
 *               results.add(new YellowRect(recognition.left, recognition.top,
 *                                         recognition.right, recognition.bottom));
 *           }
 *           return results;
 *       }
 *   }
 *
 *   4. Register with VisionManager and read results:
 *
 *   YellowRectDetector detector = new YellowRectDetector(hardwareMap.appContext);
 *   VisionManager visionManager = new VisionManager.Builder(hardwareMap)
 *       .addProcessor(detector)
 *       .build();
 *   List<YellowRect> rects = detector.getLatestResult();
 *
 * OUTPUT TENSOR FORMAT (Roboflow standard SSD export):
 *   Tensor 0: float[1][N][4]  — bounding boxes [ymin, xmin, ymax, xmax] normalized to [0,1]
 *   Tensor 1: float[1][N]     — class indices (as floats)
 *   Tensor 2: float[1][N]     — confidence scores [0.0, 1.0]
 *   Tensor 3: float[1]        — number of valid detections
 *   N = 10 for standard Roboflow mobile exports.
 *
 * CONFIDENCE:
 *   Default: 0.60. Lower to catch more detections; raise to reduce false positives.
 *   Call withMinConfidence(float) before adding the detector to VisionManager.
 *
 * @param <T> The type your interpret() method produces from raw recognitions.
 */
public abstract class KRoboflowDetector<T> extends KVisionProcessor<T> {

    private static final float DEFAULT_MIN_CONFIDENCE = 0.60f;
    private static final int MAX_DETECTIONS = 10;

    private final Context appContext;
    private final String modelFileName;
    private final String[] labels;
    private float minConfidence = DEFAULT_MIN_CONFIDENCE;

    // TFLite engine — created in onInit()
    private Interpreter interpreter;

    // Model input side length — queried from model metadata in onInit()
    private int modelInputSize;

    // Preallocated buffers — zero per-frame allocation
    private ByteBuffer inputBuffer;
    private byte[] rawPixelBuffer;
    private Object[] inputTensorWrapper;      // wraps inputBuffer, avoids per-call array alloc
    private Map<Integer, Object> outputTensorMap; // preallocated with all four output arrays

    private final float[][][] outputBoxes          = new float[1][MAX_DETECTIONS][4];
    private final float[][] outputClasses          = new float[1][MAX_DETECTIONS];
    private final float[][] outputScores           = new float[1][MAX_DETECTIONS];
    private final float[] outputNumDetections      = new float[1];

    private Mat resizedFrame;

    // Draw resources for default annotate() — allocated in onInit()
    private Paint boxPaint;
    private Paint labelPaint;

    // Camera-thread-only field — passes recognitions from detect() to annotate()
    // for the same frame. Not volatile: camera thread is the only reader/writer.
    private List<RoboflowRecognition> currentFrameRecognitions = Collections.emptyList();

    /**
     * @param appContext    Android application context. Pass hardwareMap.appContext from your OpMode.
     * @param modelFileName The .tflite filename as it appears in TeamCode/src/main/assets/
     * @param labels        Class labels in the exact order the model was trained on.
     */
    protected KRoboflowDetector(Context appContext, String modelFileName, String... labels) {
        this.appContext    = appContext;
        this.modelFileName = modelFileName;
        this.labels        = labels;
    }

    // -------------------------------------------------------------------------
    // Abstract contract — the ONE thing a subclass must implement
    // -------------------------------------------------------------------------

    /**
     * Convert raw detections into your game-specific type.
     * Called on the camera thread — keep this fast (field access, simple loops).
     *
     * @param recognitions Detections above minConfidence, sorted highest-confidence first.
     *                     Empty list if nothing was detected this frame.
     */
    protected abstract T interpret(List<RoboflowRecognition> recognitions);

    // -------------------------------------------------------------------------
    // KVisionProcessor hooks
    // -------------------------------------------------------------------------

    @Override
    protected void onInit(int frameWidth, int frameHeight) {
        try {
            interpreter = new Interpreter(loadModelFromAssets(appContext, modelFileName));
        } catch (IOException exception) {
            throw new RuntimeException("Failed to load TFLite model: " + modelFileName, exception);
        }

        int[] inputShape = interpreter.getInputTensor(0).shape(); // [1, height, width, 3]
        modelInputSize = inputShape[1];

        // Float32 input: 4 bytes per value, 3 channels (RGB), 1 batch
        inputBuffer = ByteBuffer.allocateDirect(modelInputSize * modelInputSize * 3 * 4);
        inputBuffer.order(ByteOrder.nativeOrder());

        rawPixelBuffer   = new byte[modelInputSize * modelInputSize * 3];
        resizedFrame     = new Mat();
        inputTensorWrapper = new Object[]{inputBuffer};

        outputTensorMap = new HashMap<>();
        outputTensorMap.put(0, outputBoxes);
        outputTensorMap.put(1, outputClasses);
        outputTensorMap.put(2, outputScores);
        outputTensorMap.put(3, outputNumDetections);

        boxPaint   = makeBoxPaint();
        labelPaint = makeLabelPaint();
    }

    @Override
    protected T detect(Mat frame) {
        // Step 1: Resize frame to the model's expected input dimensions
        Imgproc.resize(frame, resizedFrame, new Size(modelInputSize, modelInputSize));

        // Step 2: Fill the input buffer — normalize RGB [0, 255] → float [0.0, 1.0]
        resizedFrame.get(0, 0, rawPixelBuffer);
        inputBuffer.rewind();
        int totalPixels = modelInputSize * modelInputSize;
        for (int pixelIndex = 0; pixelIndex < totalPixels; pixelIndex++) {
            inputBuffer.putFloat((rawPixelBuffer[pixelIndex * 3]     & 0xFF) / 255.0f); // R
            inputBuffer.putFloat((rawPixelBuffer[pixelIndex * 3 + 1] & 0xFF) / 255.0f); // G
            inputBuffer.putFloat((rawPixelBuffer[pixelIndex * 3 + 2] & 0xFF) / 255.0f); // B
        }

        // Step 3: Run inference — fills outputBoxes, outputClasses, outputScores, outputNumDetections
        interpreter.runForMultipleInputsOutputs(inputTensorWrapper, outputTensorMap);

        // Step 4: Parse output tensors into RoboflowRecognition objects
        int validDetectionCount = (int) outputNumDetections[0];
        List<RoboflowRecognition> recognitions = new ArrayList<>();
        for (int detectionIndex = 0; detectionIndex < Math.min(validDetectionCount, MAX_DETECTIONS); detectionIndex++) {
            float score = outputScores[0][detectionIndex];
            if (score < minConfidence) continue;

            int classIndex = (int) outputClasses[0][detectionIndex];
            String detectionLabel = (classIndex < labels.length) ? labels[classIndex] : "Unknown";

            // Un-normalize [ymin, xmin, ymax, xmax] to full-resolution frame pixels
            float boxTop    = outputBoxes[0][detectionIndex][0] * frame.height();
            float boxLeft   = outputBoxes[0][detectionIndex][1] * frame.width();
            float boxBottom = outputBoxes[0][detectionIndex][2] * frame.height();
            float boxRight  = outputBoxes[0][detectionIndex][3] * frame.width();

            recognitions.add(new RoboflowRecognition(
                    detectionLabel, score, boxLeft, boxTop, boxRight, boxBottom));
        }

        recognitions.sort((first, second) -> Float.compare(second.confidence, first.confidence));

        // Store for annotate() — safe because detect() and annotate() run on the same thread
        currentFrameRecognitions = recognitions;
        return interpret(recognitions);
    }

    @Override
    protected void annotate(Canvas canvas, T result, DrawContext drawContext) {
        boxPaint.setStrokeWidth(STROKE_WIDTH_DP * drawContext.screenDensityScale);
        labelPaint.setTextSize(TEXT_SIZE_DP * drawContext.screenDensityScale);

        for (RoboflowRecognition recognition : currentFrameRecognitions) {
            float scaledLeft   = recognition.left   * drawContext.bitmapToCanvasScale;
            float scaledTop    = recognition.top    * drawContext.bitmapToCanvasScale;
            float scaledRight  = recognition.right  * drawContext.bitmapToCanvasScale;
            float scaledBottom = recognition.bottom * drawContext.bitmapToCanvasScale;

            canvas.drawRect(scaledLeft, scaledTop, scaledRight, scaledBottom, boxPaint);
            canvas.drawText(recognition.formattedLabel,
                    scaledLeft, scaledTop - LABEL_OFFSET_DP * drawContext.screenDensityScale,
                    labelPaint);
        }
    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /**
     * Tune the confidence threshold. Values 0.45–0.75 are typical.
     * Call before adding the detector to VisionManager.
     */
    public KRoboflowDetector<T> withMinConfidence(float confidence) {
        this.minConfidence = confidence;
        return this;
    }

    // -------------------------------------------------------------------------
    // Internal helpers
    // -------------------------------------------------------------------------

    private static MappedByteBuffer loadModelFromAssets(Context context, String modelFileName)
            throws IOException {
        try (AssetFileDescriptor assetFileDescriptor = context.getAssets().openFd(modelFileName);
             FileInputStream inputStream = new FileInputStream(assetFileDescriptor.getFileDescriptor())) {
            return inputStream.getChannel().map(
                    FileChannel.MapMode.READ_ONLY,
                    assetFileDescriptor.getStartOffset(),
                    assetFileDescriptor.getDeclaredLength()
            );
        }
    }

    private static Paint makeBoxPaint() {
        Paint paint = new Paint();
        paint.setColor(Color.WHITE);
        paint.setStyle(Paint.Style.STROKE);
        return paint;
    }

    private static Paint makeLabelPaint() {
        Paint paint = new Paint();
        paint.setColor(Color.WHITE);
        paint.setStyle(Paint.Style.FILL);
        paint.setTypeface(Typeface.MONOSPACE);
        return paint;
    }
}