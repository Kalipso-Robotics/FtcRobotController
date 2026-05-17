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
import java.util.List;

/**
 * YOLOv11 TFLite artifact detector.
 *
 * HOW TO TRAIN AND EXPORT:
 *   1. Annotate your images at roboflow.com.
 *   2. Train YOLOv11 in Google Colab using Ultralytics:
 *
 *        !pip install ultralytics roboflow
 *        from roboflow import Roboflow
 *        rf = Roboflow(api_key="YOUR_KEY")
 *        ds = rf.workspace("WS").project("PROJ").version(1).download("yolov11")
 *        from ultralytics import YOLO
 *        model = YOLO("yolo11n.pt")
 *        model.train(data=ds.location + "/data.yaml", epochs=50, imgsz=640)
 *        YOLO("runs/detect/train/weights/best.pt").export(format="tflite")
 *
 *   3. Place the .tflite file in TeamCode/src/main/assets/ as "best_float32.tflite".
 *   4. Construct in an OpMode: new TFLiteArtifactDetector(hardwareMap.appContext)
 *
 * OUTPUT TENSOR FORMAT (Ultralytics YOLOv8/v11 TFLite export):
 *   Single tensor: float[1][4 + numClasses][numAnchors]
 *     channels 0..3   — bounding box [cx, cy, w, h] normalized to [0, 1]
 *     channels 4..end — per-class confidence scores [0, 1]
 *   For yolo11n at 640x640 with 1 class: shape is [1, 5, 8400].
 */
public class TFLiteArtifactDetector extends KVisionProcessor<List<VisionRecognition>> {

    private static final String MODEL_FILE = "best_float32.tflite";
    private static final String LABEL      = "Artifact";

    private static final float DEFAULT_MIN_CONFIDENCE = 0.40f;
    private static final float NMS_IOU_THRESHOLD      = 0.45f;

    private final Context appContext;
    private float minConfidence = DEFAULT_MIN_CONFIDENCE;

    private Interpreter interpreter;

    private int modelInputSize;
    private int numClasses;
    private int numAnchors;

    private ByteBuffer inputBuffer;
    private byte[] rawPixelBuffer;
    private Object[] inputTensorWrapper;

    // Single YOLO output tensor: [1][4 + numClasses][numAnchors]
    private float[][][] outputTensor;

    private Mat resizedFrame;

    private Paint boxPaint;
    private Paint labelPaint;

    private List<VisionRecognition> currentFrameRecognitions = Collections.emptyList();

    public TFLiteArtifactDetector(Context appContext) {
        this.appContext = appContext;
    }

    @Override
    protected void onInit(int frameWidth, int frameHeight) {
        try {
            interpreter = new Interpreter(loadModelFromAssets(appContext, MODEL_FILE));
        } catch (IOException exception) {
            throw new RuntimeException("Failed to load TFLite model: " + MODEL_FILE, exception);
        }

        int[] inputShape = interpreter.getInputTensor(0).shape(); // [1, H, W, 3]
        modelInputSize = inputShape[1];

        int[] outputShape = interpreter.getOutputTensor(0).shape(); // [1, 4+nc, anchors]
        int numChannels = outputShape[1];
        numAnchors      = outputShape[2];
        numClasses      = numChannels - 4;

        inputBuffer = ByteBuffer.allocateDirect(modelInputSize * modelInputSize * 3 * 4);
        inputBuffer.order(ByteOrder.nativeOrder());

        rawPixelBuffer     = new byte[modelInputSize * modelInputSize * 3];
        resizedFrame       = new Mat();
        inputTensorWrapper = new Object[]{inputBuffer};
        outputTensor       = new float[1][numChannels][numAnchors];

        boxPaint   = makeBoxPaint();
        labelPaint = makeLabelPaint();
    }

    @Override
    protected List<VisionRecognition> detect(Mat frame) {
        Imgproc.resize(frame, resizedFrame, new Size(modelInputSize, modelInputSize));

        resizedFrame.get(0, 0, rawPixelBuffer);
        inputBuffer.rewind();
        int totalPixels = modelInputSize * modelInputSize;
        for (int pixelIndex = 0; pixelIndex < totalPixels; pixelIndex++) {
            inputBuffer.putFloat((rawPixelBuffer[pixelIndex * 3]     & 0xFF) / 255.0f); // R
            inputBuffer.putFloat((rawPixelBuffer[pixelIndex * 3 + 1] & 0xFF) / 255.0f); // G
            inputBuffer.putFloat((rawPixelBuffer[pixelIndex * 3 + 2] & 0xFF) / 255.0f); // B
        }

        interpreter.run(inputBuffer, outputTensor);

        int frameWidth  = frame.width();
        int frameHeight = frame.height();
        List<VisionRecognition> candidates = new ArrayList<>();

        for (int anchorIndex = 0; anchorIndex < numAnchors; anchorIndex++) {
            float bestScore = 0f;
            for (int classIndex = 0; classIndex < numClasses; classIndex++) {
                float score = outputTensor[0][4 + classIndex][anchorIndex];
                if (score > bestScore) bestScore = score;
            }
            if (bestScore < minConfidence) continue;

            float centerX = outputTensor[0][0][anchorIndex];
            float centerY = outputTensor[0][1][anchorIndex];
            float width   = outputTensor[0][2][anchorIndex];
            float height  = outputTensor[0][3][anchorIndex];

            float left   = (centerX - width  / 2f) * frameWidth;
            float top    = (centerY - height / 2f) * frameHeight;
            float right  = (centerX + width  / 2f) * frameWidth;
            float bottom = (centerY + height / 2f) * frameHeight;

            candidates.add(new VisionRecognition(LABEL, bestScore, left, top, right, bottom));
        }

        candidates.sort((first, second) -> Float.compare(second.confidence, first.confidence));
        List<VisionRecognition> recognitions = nonMaxSuppress(candidates);

        currentFrameRecognitions = recognitions;
        return recognitions;
    }

    @Override
    protected void annotate(Canvas canvas, List<VisionRecognition> result, DrawContext drawContext) {
        boxPaint.setStrokeWidth(STROKE_WIDTH_DP * drawContext.screenDensityScale);
        labelPaint.setTextSize(TEXT_SIZE_DP * drawContext.screenDensityScale);

        for (VisionRecognition recognition : currentFrameRecognitions) {
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

    /** Tune the confidence threshold. Values 0.30–0.60 are typical for YOLO. */
    public TFLiteArtifactDetector withMinConfidence(float confidence) {
        this.minConfidence = confidence;
        return this;
    }

    private static List<VisionRecognition> nonMaxSuppress(List<VisionRecognition> sortedByScore) {
        List<VisionRecognition> kept = new ArrayList<>();
        for (VisionRecognition candidate : sortedByScore) {
            boolean suppressed = false;
            for (VisionRecognition keeper : kept) {
                if (!candidate.label.equals(keeper.label)) continue;
                if (intersectionOverUnion(candidate, keeper) > NMS_IOU_THRESHOLD) {
                    suppressed = true;
                    break;
                }
            }
            if (!suppressed) kept.add(candidate);
        }
        return kept;
    }

    private static float intersectionOverUnion(VisionRecognition a, VisionRecognition b) {
        float interLeft   = Math.max(a.left,   b.left);
        float interTop    = Math.max(a.top,    b.top);
        float interRight  = Math.min(a.right,  b.right);
        float interBottom = Math.min(a.bottom, b.bottom);

        float interWidth  = Math.max(0f, interRight  - interLeft);
        float interHeight = Math.max(0f, interBottom - interTop);
        float intersection = interWidth * interHeight;
        if (intersection <= 0f) return 0f;

        float areaA = (a.right - a.left) * (a.bottom - a.top);
        float areaB = (b.right - b.left) * (b.bottom - b.top);
        return intersection / (areaA + areaB - intersection);
    }

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
