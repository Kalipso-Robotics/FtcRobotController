package org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblobbing;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.KVisionProcessor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionRecognition;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Abstract base for all HSV color-blob-based processors.
 *
 * HOW IT WORKS:
 *   1. Downsamples the 640x480 capture to 320x240 for fast OpenCV work.
 *   2. Gaussian-blurs the downsampled frame to suppress pixel noise before thresholding.
 *   3. Converts to HSV — more robust than RGB under changing venue lighting.
 *   4. Thresholds each ColorChannel to produce a binary mask.
 *   5. Applies morphological OPEN (removes noise) then CLOSE (fills sphere holes).
 *   6. Finds external contours, filters by area and circularity.
 *   7. Scales surviving bounding boxes back to full 640x480 resolution.
 *   8. Returns a List<KVisionRecognition> (concretely DetectedBlob) sorted largest-first.
 *
 * HOW TO CREATE A NEW PROCESSOR:
 *   Just define your color channels. Everything else is handled here.
 *
 *   public class YellowBallProcessor extends KColorBlobProcessor {
 *       @Override
 *       protected ColorChannel[] defineChannels() {
 *           return new ColorChannel[] {
 *               new ColorChannel(
 *                   new Scalar(20, 100, 100),   // HSV lower bound
 *                   new Scalar(35, 255, 255),   // HSV upper bound
 *                   "Yellow",                    // label shown in telemetry
 *                   Color.YELLOW                 // overlay color on DS stream
 *               )
 *           };
 *       }
 *   }
 *
 * HSV REFERENCE (OpenCV scale: H 0-180, S 0-255, V 0-255):
 *   Red      ~  0-10  and 170-180
 *   Orange   ~ 10-20
 *   Yellow   ~ 20-35
 *   Green    ~ 40-85
 *   Cyan     ~ 85-100
 *   Blue     ~ 100-130
 *   Purple   ~ 117-155
 *   Always set S_min > 50 and V_min > 40 to reject grey/black/white noise.
 */
public abstract class KColorBlobProcessor extends KVisionProcessor<List<VisionRecognition>> {

    public static class ColorChannel {
        public final Scalar hsvLowerBound;
        public final Scalar hsvUpperBound;
        /** Label used in DetectedBlob.label and telemetry. */
        public final String label;
        /** Android color int for bounding box and text on the DS stream. */
        public final int overlayColor;

        Mat mask;
        Paint boxPaint;
        Paint textPaint;

        public ColorChannel(Scalar hsvLowerBound, Scalar hsvUpperBound,
                            String label, int overlayColor) {
            this.hsvLowerBound = hsvLowerBound;
            this.hsvUpperBound = hsvUpperBound;
            this.label = label;
            this.overlayColor = overlayColor;
        }
    }

    protected double minContourArea = 250;
    protected double maxContourArea = 30_000;
    // REVIEW: measured spheres come out around 0.85-0.90, so 0.55 is loose enough
    // to admit partially-merged clutter. 0.75 is the suggested value -- try it live
    // from ArtifactDetectionTest before changing it here.
    protected double minCircularity = 0.55;

    /**
     * Expected bounding-box width/height ratio for the target object.
     *
     * A sphere subtending angle a projects to fx*a pixels wide by fy*a pixels tall,
     * so the expected ratio is fx/fy, NOT 1.0. For the Arducam at 640x480
     * (fx=444.14, fy=532.28) that is 0.835.
     *
     * This doubles as an intrinsics check: if real balls consistently measure ~1.0
     * here, the 1280x800 -> 640x480 rescale in CameraIntrinsics is wrong and fy is
     * off by ~19%. See fit_intrinsics.py.
     */
    protected double expectedAspect = 444.14195 / 532.27560; // = 0.8344

    /**
     * Half-width of the accepted aspect band around expectedAspect.
     *
     * 0 (or negative) DISABLES the gate entirely -- the default, so this class
     * behaves exactly as before until the gate is deliberately switched on.
     * REVIEW: 0.30 is the suggested value. Turn it on live from the dashboard in
     * ArtifactDetectionTest, watch the ASPECT rejections in the snapshots, then
     * decide whether to make it the default here.
     */
    protected double aspectTolerance = 0;

    public void setMinContourArea(double v)  { this.minContourArea = v; }
    public void setMaxContourArea(double v)  { this.maxContourArea = v; }
    public void setMinCircularity(double v)  { this.minCircularity = v; }
    public void setExpectedAspect(double v)  { this.expectedAspect = v; }
    public void setAspectTolerance(double v) { this.aspectTolerance = v; }

    public double getMinContourArea()  { return minContourArea; }
    public double getMaxContourArea()  { return maxContourArea; }
    public double getMinCircularity()  { return minCircularity; }
    public double getExpectedAspect()  { return expectedAspect; }
    public double getAspectTolerance() { return aspectTolerance; }

    /**
     * Real-world diameter (mm) of the object this processor detects, e.g. a game ball.
     * 0 = unset — size-based ranging (CameraIntrinsics.calculateRobotFramePosFromSize)
     * is unavailable until a subclass sets this. Subclasses set it directly.
     */
    protected double objectDiameterMM = 0;

    public double getObjectDiameterMM() { return objectDiameterMM; }

    /** Gaussian blur kernel size (must be odd). Larger = more smoothing, less noise. */
    protected int gaussianKernelSize = 5;

    // Diagnostic counters — written on camera thread, read from main thread via getDiagnosticSummary()
    private volatile int diagFrameCount = 0;
    private volatile int diagRawContours = 0;
    private volatile int diagAreaRejected = 0;
    private volatile int diagCircRejected = 0;
    private volatile int diagAspectRejected = 0;

    // Snapshot-to-disk debug feature
    private volatile int snapshotEveryNFrames = 0; // 0 = disabled
    private File snapshotDir;
    private Mat snapshotBgrBuffer;
    private final long snapshotSessionId = System.currentTimeMillis();

    // Per-frame rejected-contour log for debug snapshots.
    private static class RejectedContour {
        final Rect rect; final double area; final double circularity;
        final String label; final String reason;
        RejectedContour(Rect r, double a, double c, String l, String reason) {
            this.rect = r; this.area = a; this.circularity = c; this.label = l; this.reason = reason;
        }
    }
    private final List<RejectedContour> lastRejected = new ArrayList<>();

    private static final int PROCESSING_WIDTH  = 320;
    private static final int PROCESSING_HEIGHT = 240;

    private Mat downsampledFrame;
    private Mat blurredFrame;
    private Mat hsvFrame;
    private Mat morphologyBuffer;
    private Mat noiseRemovalKernel;
    private Mat holeFillingKernel;
    private Mat contourHierarchy;
    private MatOfPoint2f reusableContourBuffer;
    private final List<MatOfPoint> reusableContourList = new ArrayList<>();

    private double widthScaleFactor;
    private double heightScaleFactor;
    private Paint crosshairPaint;

    /**
     * Define the color(s) this processor should detect.
     * Called once during initialisation. Return one ColorChannel per color.
     */
    protected abstract ColorChannel[] defineChannels();

    private ColorChannel[] channels;

    @Override
    protected void onInit(int frameWidth, int frameHeight) {
        widthScaleFactor  = (double) frameWidth  / PROCESSING_WIDTH;
        heightScaleFactor = (double) frameHeight / PROCESSING_HEIGHT;

        channels = defineChannels();

        downsampledFrame      = new Mat();
        blurredFrame          = new Mat();
        hsvFrame              = new Mat();
        morphologyBuffer      = new Mat();
        reusableContourBuffer = new MatOfPoint2f();
        noiseRemovalKernel    = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(3, 3));
        holeFillingKernel     = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(7, 7));
        contourHierarchy      = new Mat();

        for (ColorChannel channel : channels) {
            channel.mask      = new Mat();
            channel.boxPaint  = makeStrokePaint(channel.overlayColor);
            channel.textPaint = makeMonospaceTextPaint(channel.overlayColor);
        }
        crosshairPaint = makeStrokePaint(Color.WHITE);
        crosshairPaint.setStrokeWidth(CROSSHAIR_STROKE_DP);
    }

    @Override
    protected List<VisionRecognition> detect(Mat frame) {
        diagFrameCount++;
        diagRawContours = 0;
        diagAreaRejected = 0;
        diagCircRejected = 0;
        diagAspectRejected = 0;
        lastRejected.clear();

        Imgproc.resize(frame, downsampledFrame,
                new Size(PROCESSING_WIDTH, PROCESSING_HEIGHT), 0, 0, Imgproc.INTER_LINEAR);
        Imgproc.GaussianBlur(downsampledFrame, blurredFrame,
                new Size(gaussianKernelSize, gaussianKernelSize), 0);
        Imgproc.cvtColor(blurredFrame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        List<VisionRecognition> allBlobs = new ArrayList<>();
        for (ColorChannel channel : channels) {
            Core.inRange(hsvFrame, channel.hsvLowerBound, channel.hsvUpperBound, channel.mask);
            allBlobs.addAll(extractBlobsFromMask(channel.mask, channel.label));
        }

        allBlobs.sort((first, second) -> Double.compare(second.getArea(), first.getArea()));

        logDetectionStream(allBlobs);
        maybeSaveSnapshot(frame, allBlobs);

        return allBlobs;
    }

    /**
     * Always-on per-frame detection log so you can see in real time whether
     * the camera ever sees a ball. Strategy:
     *   - If any blob was accepted this frame: log every frame with per-blob detail.
     *   - If nothing accepted: log every 30 frames with pipeline counters so the
     *     stream isn't silent but doesn't drown logcat either.
     *
     * Tag: KColorBlobProcessor (separate from VisionRoundTrip). Filter with:
     *   adb logcat | grep KColorBlobProcessor
     */
    private void logDetectionStream(List<VisionRecognition> blobs) {
        if (!blobs.isEmpty()) {
            StringBuilder sb = new StringBuilder();
            sb.append(String.format(Locale.US, "f%d SAW %d:", diagFrameCount, blobs.size()));
            for (VisionRecognition r : blobs) {
                if (!(r instanceof DetectedBlob)) continue;
                DetectedBlob b = (DetectedBlob) r;
                double aspect = (b.getHeight() > 0) ? b.getWidth() / b.getHeight() : 0.0;
                sb.append(String.format(Locale.US,
                        " [%s a=%.0f c=%.2f wh=%.0fx%.0f ar=%.2f px=(%.0f,%.0f)]",
                        b.label, b.getArea(), b.getCircularity(),
                        b.getWidth(), b.getHeight(), aspect,
                        b.center.getX(), b.center.getY()));
            }
            android.util.Log.i("KColorBlobProcessor", sb.toString());
        } else if (diagFrameCount % 30 == 0) {
            android.util.Log.i("KColorBlobProcessor", String.format(Locale.US,
                    "f%d NONE  raw=%d areaRej=%d circRej=%d aspectRej=%d "
                            + "minArea=%.0f minCirc=%.2f aspect=%.2f+/-%.2f",
                    diagFrameCount, diagRawContours, diagAreaRejected, diagCircRejected,
                    diagAspectRejected, minContourArea, minCircularity,
                    expectedAspect, aspectTolerance));
        }
    }

    /**
     * Enable periodic JPEG dumps of the raw camera frame (with bounding boxes
     * drawn on it) to /sdcard/FIRST/data/vision_snapshots/. Set to 0 to disable.
     *
     * Pull files off the Control Hub:
     *   adb pull /sdcard/FIRST/data/vision_snapshots ~/Desktop
     * Or use the Control Hub Manage page → Manage Files.
     *
     * Each session writes to a subdir named with the session start epoch-ms,
     * so previous runs aren't overwritten.
     */
    public void enableSnapshotSaving(int everyNFrames) {
        if (everyNFrames > 0) {
            File base = new File(AppUtil.ROOT_FOLDER, "data/vision_snapshots/" + snapshotSessionId);
            if (!base.exists() && !base.mkdirs()) {
                android.util.Log.w("KColorBlobProcessor", "Could not create snapshot dir: " + base);
            }
            snapshotDir = base;
            android.util.Log.i("KColorBlobProcessor", "Snapshot saving ON every " + everyNFrames
                    + " frames → " + base.getAbsolutePath());
        } else {
            snapshotDir = null;
        }
        snapshotEveryNFrames = everyNFrames;
    }

    private void drawHeaderBanner(Mat bgr, int accepted, int rejected) {
        int bannerHeight = 38;
        // Solid black band across the top of the image so text is readable.
        Imgproc.rectangle(bgr,
                new org.opencv.core.Point(0, 0),
                new org.opencv.core.Point(bgr.cols(), bannerHeight),
                new Scalar(0, 0, 0), -1);

        Position pose = null;
        try {
            pose = SharedData.getOdometryWheelIMUPosition();
        } catch (Throwable ignored) { /* opmode may not be running yet */ }

        long elapsedMs = System.currentTimeMillis() - snapshotSessionId;
        String line1 = String.format(Locale.US,
                "f%06d  t=%6.2fs  accepted=%d  rejected=%d",
                diagFrameCount, elapsedMs / 1000.0, accepted, rejected);
        String line2 = (pose == null)
                ? "pose=N/A"
                : String.format(Locale.US,
                        "pose=(%.0f,%.0f,%.1f deg) raw=%d areaRej=%d circRej=%d aspectRej=%d",
                        pose.getX(), pose.getY(), Math.toDegrees(pose.getTheta()),
                        diagRawContours, diagAreaRejected, diagCircRejected, diagAspectRejected);

        Scalar white = new Scalar(255, 255, 255);
        Imgproc.putText(bgr, line1, new org.opencv.core.Point(4, 14),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.45, white, 1);
        Imgproc.putText(bgr, line2, new org.opencv.core.Point(4, 32),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, white, 1);
    }

    private void maybeSaveSnapshot(Mat rgbFrame, List<VisionRecognition> blobs) {
        int n = snapshotEveryNFrames;
        if (n <= 0 || snapshotDir == null) return;
        if (diagFrameCount % n != 0) return;

        try {
            if (snapshotBgrBuffer == null) snapshotBgrBuffer = new Mat();
            // OpenCV's imwrite expects BGR; the FTC frame is RGB.
            Imgproc.cvtColor(rgbFrame, snapshotBgrBuffer, Imgproc.COLOR_RGB2BGR);

            // Accepted blobs: purple/green boxes.
            for (VisionRecognition r : blobs) {
                if (!(r instanceof DetectedBlob)) continue;
                DetectedBlob b = (DetectedBlob) r;
                Scalar color = "Purple".equals(b.label)
                        ? new Scalar(255, 0, 180)  // BGR ~ purple
                        : new Scalar(60, 220, 0);  // BGR ~ green
                Imgproc.rectangle(snapshotBgrBuffer,
                        new org.opencv.core.Point(b.left, b.top),
                        new org.opencv.core.Point(b.right, b.bottom),
                        color, 2);
                Imgproc.putText(snapshotBgrBuffer,
                        String.format(Locale.US, "%s a=%.0f c=%.2f",
                                b.label, b.getArea(), b.getCircularity()),
                        new org.opencv.core.Point(b.left, Math.max(b.top - 4, 12)),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            }

            // Rejected blobs: red boxes with the rejection reason + area/circ.
            // These are contours that DID match the HSV mask but failed the
            // area or circularity filter — exactly the data needed to tune thresholds.
            Scalar rejectedColor = new Scalar(0, 0, 255); // BGR ~ red
            int rejectedCount = 0;
            for (RejectedContour rc : lastRejected) {
                Imgproc.rectangle(snapshotBgrBuffer,
                        new org.opencv.core.Point(rc.rect.x, rc.rect.y),
                        new org.opencv.core.Point(rc.rect.x + rc.rect.width, rc.rect.y + rc.rect.height),
                        rejectedColor, 1);
                Imgproc.putText(snapshotBgrBuffer,
                        String.format(Locale.US, "REJ %s %s a=%.0f c=%.2f",
                                rc.label, rc.reason, rc.area, rc.circularity),
                        new org.opencv.core.Point(rc.rect.x, Math.max(rc.rect.y - 4, 12)),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, rejectedColor, 1);
                rejectedCount++;
            }

            // Top banner with frame#, timestamp, robot pose, and detection counts.
            // Lets you correlate a snapshot to a log line ("frames=N") and to the
            // pose the robot believed it was at when the camera captured the frame.
            drawHeaderBanner(snapshotBgrBuffer, blobs.size(), rejectedCount);

            String name = String.format(Locale.US, "f%06d_n%d_r%d.jpg",
                    diagFrameCount, blobs.size(), rejectedCount);
            Imgcodecs.imwrite(new File(snapshotDir, name).getAbsolutePath(), snapshotBgrBuffer);
        } catch (Throwable t) {
            // Never let snapshot IO take down the vision pipeline.
            android.util.Log.w("KColorBlobProcessor", "Snapshot save failed: " + t);
        }
    }

    @Override
    public String getDiagnosticSummary() {
        return String.format(Locale.US,
                "frames=%d rawContours=%d areaRej=%d circRej=%d aspectRej=%d "
                        + "minArea=%.0f minCirc=%.2f aspect=%.2f+/-%.2f",
                diagFrameCount, diagRawContours, diagAreaRejected, diagCircRejected,
                diagAspectRejected, minContourArea, minCircularity,
                expectedAspect, aspectTolerance);
    }

    @Override
    protected void annotate(Canvas canvas, List<VisionRecognition> blobs, DrawContext drawContext) {
        for (VisionRecognition recognition : blobs) {
            if (!(recognition instanceof DetectedBlob)) continue;
            DetectedBlob blob = (DetectedBlob) recognition;

            ColorChannel channel = channelFor(blob.label);
            if (channel == null) continue;

            channel.boxPaint.setStrokeWidth(STROKE_WIDTH_DP * drawContext.screenDensityScale);
            channel.textPaint.setTextSize(TEXT_SIZE_DP * drawContext.screenDensityScale);

            float boxLeft   = blob.left   * drawContext.bitmapToCanvasScale;
            float boxTop    = blob.top    * drawContext.bitmapToCanvasScale;
            float boxRight  = blob.right  * drawContext.bitmapToCanvasScale;
            float boxBottom = blob.bottom * drawContext.bitmapToCanvasScale;
            canvas.drawRect(boxLeft, boxTop, boxRight, boxBottom, channel.boxPaint);
            canvas.drawText(blob.toString(), boxLeft,
                    boxTop - LABEL_OFFSET_DP * drawContext.screenDensityScale, channel.textPaint);

            float centerX = (float) blob.center.getX() * drawContext.bitmapToCanvasScale;
            float centerY = (float) blob.center.getY() * drawContext.bitmapToCanvasScale;
            canvas.drawLine(centerX - drawContext.crosshairArmLength, centerY,
                            centerX + drawContext.crosshairArmLength, centerY, crosshairPaint);
            canvas.drawLine(centerX, centerY - drawContext.crosshairArmLength,
                            centerX, centerY + drawContext.crosshairArmLength, crosshairPaint);
        }
    }

    /** Largest blob across all channels. Null if nothing detected. */
    public DetectedBlob getLargestBlob() {
        List<VisionRecognition> snapshot = getLatestResult();
        if (snapshot == null || snapshot.isEmpty()) return null;
        VisionRecognition first = snapshot.get(0);
        return (first instanceof DetectedBlob) ? (DetectedBlob) first : null;
    }

    /** Largest blob matching the given channel label. Null if not detected. */
    public DetectedBlob getLargestBlobByLabel(String colorLabel) {
        List<VisionRecognition> snapshot = getLatestResult();
        if (snapshot == null) return null;
        for (VisionRecognition recognition : snapshot) {
            if (colorLabel.equals(recognition.label) && recognition instanceof DetectedBlob) {
                return (DetectedBlob) recognition;
            }
        }
        return null;
    }

    public boolean hasBlobWithLabel(String colorLabel) {
        return getLargestBlobByLabel(colorLabel) != null;
    }

    /**
     * Live color channels, for dashboard tuning. Scalar.val[] is mutable, so a
     * tuning OpMode can write bounds in place:
     *   getChannels()[0].hsvLowerBound.val[1] = newSaturationMin;
     * Returns null before onInit() has run. Not for use in competition code.
     */
    public ColorChannel[] getChannels() { return channels; }

    // Raw diagnostic counters, for CSV logging by tuning OpModes.
    public int getDiagFrameCount()     { return diagFrameCount; }
    public int getDiagRawContours()    { return diagRawContours; }
    public int getDiagAreaRejected()   { return diagAreaRejected; }
    public int getDiagCircRejected()   { return diagCircRejected; }
    public int getDiagAspectRejected() { return diagAspectRejected; }

    private List<DetectedBlob> extractBlobsFromMask(Mat colorMask, String colorLabel) {
        Imgproc.morphologyEx(colorMask, morphologyBuffer, Imgproc.MORPH_OPEN, noiseRemovalKernel);
        Imgproc.morphologyEx(morphologyBuffer, morphologyBuffer, Imgproc.MORPH_CLOSE, holeFillingKernel);

        reusableContourList.clear();
        Imgproc.findContours(morphologyBuffer, reusableContourList, contourHierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        diagRawContours += reusableContourList.size();

        List<DetectedBlob> blobs = new ArrayList<>();
        for (MatOfPoint contour : reusableContourList) {
            Rect boundingBox = Imgproc.boundingRect(contour);
            double area = Imgproc.contourArea(contour);
            if (area < minContourArea || area > maxContourArea) {
                diagAreaRejected++;
                String reason = (area < minContourArea) ? "AREA<min" : "AREA>max";
                lastRejected.add(new RejectedContour(
                        scaleRectToFullResolution(boundingBox),
                        area, 0.0, colorLabel, reason));
                contour.release();
                continue;
            }

            reusableContourBuffer.fromArray(contour.toArray());
            double circularity = computeCircularity(area, Imgproc.arcLength(reusableContourBuffer, true));
            if (circularity < minCircularity) {
                diagCircRejected++;
                lastRejected.add(new RejectedContour(
                        scaleRectToFullResolution(boundingBox),
                        area, circularity, colorLabel, "CIRC<min"));
                contour.release();
                continue;
            }

            // A ball is round. Elongated clutter (tape lines, bumpers, wall stripes)
            // can survive the circularity gate once MORPH_CLOSE merges nearby specks,
            // but it cannot survive an aspect-ratio gate. Downsample scaling is 2x on
            // both axes, so the ratio is identical before and after rescaling.
            //
            // OPT-IN: disabled while aspectTolerance is 0, which is the default.
            // Enable it live from ArtifactDetectionTest before adopting it here.
            double aspect = (boundingBox.height > 0)
                    ? (double) boundingBox.width / boundingBox.height : 0.0;
            if (aspectTolerance > 0 && Math.abs(aspect - expectedAspect) > aspectTolerance) {
                diagAspectRejected++;
                lastRejected.add(new RejectedContour(
                        scaleRectToFullResolution(boundingBox),
                        area, circularity, colorLabel,
                        String.format(Locale.US, "ASPECT=%.2f", aspect)));
                contour.release();
                continue;
            }

            blobs.add(new DetectedBlob(
                scaleRectToFullResolution(boundingBox),
                area, circularity, colorLabel
            ));
            contour.release();
        }
        return blobs;
    }

    private ColorChannel channelFor(String colorLabel) {
        for (ColorChannel channel : channels) {
            if (channel.label.equals(colorLabel)) return channel;
        }
        return null;
    }

    private static double computeCircularity(double area, double perimeter) {
        return (perimeter > 0) ? (4.0 * Math.PI * area) / (perimeter * perimeter) : 0;
    }

    private Rect scaleRectToFullResolution(Rect downsampledRect) {
        return new Rect(
            (int)(downsampledRect.x      * widthScaleFactor),
            (int)(downsampledRect.y      * heightScaleFactor),
            (int)(downsampledRect.width  * widthScaleFactor),
            (int)(downsampledRect.height * heightScaleFactor)
        );
    }

    private static Paint makeStrokePaint(int color) {
        Paint paint = new Paint();
        paint.setColor(color);
        paint.setStyle(Paint.Style.STROKE);
        return paint;
    }

    private static Paint makeMonospaceTextPaint(int color) {
        Paint paint = new Paint();
        paint.setColor(color);
        paint.setStyle(Paint.Style.FILL);
        paint.setTypeface(Typeface.MONOSPACE);
        return paint;
    }
}
