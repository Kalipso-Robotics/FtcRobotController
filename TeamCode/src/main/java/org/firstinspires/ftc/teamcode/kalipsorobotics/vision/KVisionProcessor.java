package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

/**
 * Abstract base class for all custom vision processors on this robot.
 *
 * WHY THIS EXISTS:
 *   Writing a new VisionProcessor from scratch requires implementing three
 *   interface methods (init, processFrame, onDrawFrame), managing volatile
 *   state for thread safety, and wiring everything to VisionManager. This
 *   base class handles all of that boilerplate so a new processor only
 *   needs to answer two questions:
 *     1. Given a camera frame, what did I detect?      → detect()
 *     2. How do I draw the result on the DS stream?   → annotate()
 *
 * THREADING:
 *   processFrame() and onDrawFrame() run on VisionPortal's camera thread.
 *   The robot main loop calls getLatestResult() which reads a volatile
 *   reference — non-blocking, never stalls the main loop.
 *
 * USAGE — create a new processor in three steps:
 *
 *   public class MyProcessor extends KVisionProcessor<MyResult> {
 *
 *       // Step 1: allocate your Mats / resources once
 *       @Override protected void onInit(int frameWidth, int frameHeight) { ... }
 *
 *       // Step 2: run your detection and return the result
 *       @Override protected MyResult detect(Mat frame) { ... }
 *
 *       // Step 3 (optional): draw on the Driver Station preview
 *       @Override protected void annotate(Canvas canvas, MyResult result, DrawContext ctx) { ... }
 *   }
 *
 * @param <T> The type this processor produces each frame (e.g. List<DetectedBlob>).
 */
public abstract class KVisionProcessor<T> implements VisionProcessor {

    // -------------------------------------------------------------------------
    // Draw constants — available to all subclasses
    // -------------------------------------------------------------------------
    protected static final float STROKE_WIDTH_DP    = 3f;
    protected static final float TEXT_SIZE_DP       = 13f;
    protected static final float CROSSHAIR_ARM_DP   = 8f;
    protected static final float CROSSHAIR_STROKE_DP = 2f;
    protected static final float LABEL_OFFSET_DP    = 4f;

    // -------------------------------------------------------------------------
    // Thread-safe result storage
    // -------------------------------------------------------------------------
    private volatile T latestResult;

    // -------------------------------------------------------------------------
    // Abstract contract — subclasses implement these
    // -------------------------------------------------------------------------

    /**
     * Called once when the VisionPortal is built. Allocate Mats and Paint
     * objects here — never inside detect() or annotate().
     */
    protected void onInit(int frameWidth, int frameHeight) {}

    /**
     * Run your detection logic and return the result.
     * Called on the camera thread — do not touch robot hardware here.
     * @param frame The raw RGB camera frame at full capture resolution.
     */
    protected abstract T detect(Mat frame);

    /**
     * Draw annotations onto the Driver Station camera preview.
     * Called on the camera thread immediately after detect().
     * Default implementation draws nothing — override when needed.
     */
    protected void annotate(Canvas canvas, T result, DrawContext drawContext) {}

    // -------------------------------------------------------------------------
    // VisionProcessor — sealed so subclasses cannot break the threading model
    // -------------------------------------------------------------------------

    @Override
    public final void init(int width, int height, CameraCalibration calibration) {
        onInit(width, height);
    }

    @Override
    public final Object processFrame(Mat frame, long captureTimeNanos) {
        T result = detect(frame);
        latestResult = result;
        return result; // passed to onDrawFrame as userContext
    }

    @Override
    @SuppressWarnings("unchecked")
    public final void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                  float bitmapToCanvasScale, float screenDensityScale,
                                  Object userContext) {
        T result = (T) userContext;
        if (result == null) return;
        annotate(canvas, result, new DrawContext(bitmapToCanvasScale, screenDensityScale));
    }

    // -------------------------------------------------------------------------
    // Public API — called from robot/action thread
    // -------------------------------------------------------------------------

    /**
     * The most recent detection result. Null until the first frame is processed.
     * Non-blocking — safe to call every loop iteration.
     */
    public T getLatestResult() { return latestResult; }

    /** True once at least one frame has been processed. */
    public boolean hasResult() { return latestResult != null; }

    // -------------------------------------------------------------------------
    // DrawContext — passed to annotate() so subclasses never need to touch the
    //              raw scale parameters from the VisionProcessor interface.
    // -------------------------------------------------------------------------

    protected static class DrawContext {
        /** Multiplier to convert bitmap pixels → canvas pixels for drawRect/drawText. */
        public final float bitmapToCanvasScale;
        /** Screen density multiplier for stroke widths and text sizes. */
        public final float screenDensityScale;
        /** Pre-computed crosshair arm length in canvas pixels. */
        public final float crosshairArmLength;

        DrawContext(float bitmapToCanvasScale, float screenDensityScale) {
            this.bitmapToCanvasScale = bitmapToCanvasScale;
            this.screenDensityScale = screenDensityScale;
            this.crosshairArmLength = CROSSHAIR_ARM_DP * screenDensityScale;
        }
    }
}
