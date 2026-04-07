package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.tfod.Recognition;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Mat;

import java.util.Collections;
import java.util.List;

/**
 * Abstract base class for ML-based object detectors trained with Roboflow.
 *
 * HOW TO USE WITH ROBOFLOW:
 *   1. Annotate your images at roboflow.com and train a model.
 *   2. Export as "TFLite" format.
 *   3. Place the .tflite file in TeamCode/src/main/assets/
 *   4. Create a subclass and implement interpret():
 *
 *   public class YellowRectDetector extends KRoboflowDetector<List<YellowRect>> {
 *
 *       public YellowRectDetector() {
 *           super("yellow_rect.tflite", "yellow_rect");
 *       }
 *
 *       @Override
 *       protected List<YellowRect> interpret(List<Recognition> recognitions) {
 *           List<YellowRect> results = new ArrayList<>();
 *           for (Recognition r : recognitions) {
 *               results.add(new YellowRect(
 *                   r.getLeft(), r.getTop(), r.getWidth(), r.getHeight()
 *               ));
 *           }
 *           return results;
 *       }
 *   }
 *
 *   5. Register with VisionManager and read results:
 *
 *   YellowRectDetector detector = new YellowRectDetector();
 *   VisionManager visionManager = new VisionManager.Builder(hardwareMap)
 *       .addProcessor(detector)
 *       .build();
 *
 *   List<YellowRect> rects = detector.getLatestResult();
 *
 * CONFIDENCE:
 *   Default minimum confidence is 0.60. Lower it if detections are missed.
 *   Raise it to reduce false positives in noisy environments.
 *   Call withMinConfidence(float) in your constructor before the portal starts.
 *
 * THREADING:
 *   getLatestResult() is safe to call every robot loop iteration — it reads
 *   from TfodProcessor's internal buffer which is already thread-safe.
 *
 * @param <T> The type your interpret() method produces from raw recognitions.
 */
public abstract class KRoboflowDetector<T> implements VisionProcessor {

    private static final float DEFAULT_MIN_CONFIDENCE = 0.60f;

    private final TfodProcessor tfodProcessor;

    /**
     * @param modelFileName  The .tflite filename as it appears in assets/ (no path prefix).
     * @param labels         Every class label your model was trained on, in the same order
     *                       as the model's output layer.
     */
    protected KRoboflowDetector(String modelFileName, String... labels) {
        tfodProcessor = new TfodProcessor.Builder()
                .setModelFileName(modelFileName)
                .setModelLabels(labels)
                .setMinResultConfidence(DEFAULT_MIN_CONFIDENCE)
                .build();
    }

    // -------------------------------------------------------------------------
    // Abstract contract — the ONE thing you must implement
    // -------------------------------------------------------------------------

    /**
     * Convert raw TFOD recognitions into your game-specific type.
     * Called on the robot main thread when getLatestResult() is called.
     * Filter by confidence here if the default threshold is not enough.
     *
     * @param recognitions The raw detections from TfodProcessor. May be empty.
     */
    protected abstract T interpret(List<Recognition> recognitions);

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /**
     * Interpret the latest frame's detections and return your result type.
     * Non-blocking — safe to call every robot loop iteration.
     */
    public T getLatestResult() {
        List<Recognition> recognitions = tfodProcessor.getRecognitions();
        if (recognitions == null) return interpret(Collections.emptyList());
        return interpret(recognitions);
    }

    public boolean hasResult() {
        List<Recognition> recognitions = tfodProcessor.getRecognitions();
        return recognitions != null && !recognitions.isEmpty();
    }

    /**
     * Tune the confidence threshold. Values between 0.45 and 0.75 are typical.
     * Call this before adding the detector to VisionManager.
     */
    public KRoboflowDetector<T> withMinConfidence(float minConfidence) {
        tfodProcessor.setMinResultConfidence(minConfidence);
        return this;
    }

    // -------------------------------------------------------------------------
    // VisionProcessor delegation — TfodProcessor handles all of this internally.
    // We delegate so this class can be added directly to VisionManager.
    // -------------------------------------------------------------------------

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        tfodProcessor.init(width, height, calibration);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return tfodProcessor.processFrame(frame, captureTimeNanos);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float bitmapToCanvasScale, float screenDensityScale,
                            Object userContext) {
        // TfodProcessor draws its own bounding boxes — no extra work needed.
        tfodProcessor.onDrawFrame(canvas, onscreenWidth, onscreenHeight,
                bitmapToCanvasScale, screenDensityScale, userContext);
    }
}
