package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

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
 *   8. Returns a List<DetectedBlob> sorted largest-first.
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
public abstract class KColorBlobProcessor extends KVisionProcessor<List<DetectedBlob>> {

    // -------------------------------------------------------------------------
    // ColorChannel — one entry per color you want to detect
    // -------------------------------------------------------------------------

    public static class ColorChannel {
        public final Scalar hsvLowerBound;
        public final Scalar hsvUpperBound;
        /** Label used in DetectedBlob.colorLabel and telemetry. */
        public final String label;
        /** Android color int for bounding box and text on the DS stream. */
        public final int overlayColor;

        // Allocated in onInit()
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

    // -------------------------------------------------------------------------
    // Detection thresholds — override in subclass to change sensitivity
    // -------------------------------------------------------------------------
    protected double minContourArea    = 250;
    protected double maxContourArea    = 30_000;
    protected double minCircularity    = 0.55;

    /** Gaussian blur kernel size (must be odd). Larger = more smoothing, less noise. */
    protected int gaussianKernelSize = 5;

    // -------------------------------------------------------------------------
    // Processing resolution
    // -------------------------------------------------------------------------
    private static final int PROCESSING_WIDTH  = 320;
    private static final int PROCESSING_HEIGHT = 240;

    // -------------------------------------------------------------------------
    // Reusable OpenCV Mats — allocated once, never inside detect()
    // -------------------------------------------------------------------------
    private Mat downsampledFrame;
    private Mat blurredFrame;
    private Mat hsvFrame;
    private Mat morphologyBuffer;
    private Mat noiseRemovalKernel;  // MORPH_OPEN  — erode then dilate
    private Mat holeFillingKernel;   // MORPH_CLOSE — dilate then erode
    private Mat contourHierarchy;
    private MatOfPoint2f reusableContourBuffer;
    private final List<MatOfPoint> reusableContourList = new ArrayList<>();

    private double widthScaleFactor;
    private double heightScaleFactor;
    private Paint crosshairPaint;

    // -------------------------------------------------------------------------
    // Abstract contract — the ONE thing a subclass must implement
    // -------------------------------------------------------------------------

    /**
     * Define the color(s) this processor should detect.
     * Called once during initialisation. Return one ColorChannel per color.
     */
    protected abstract ColorChannel[] defineChannels();

    // Populated from defineChannels() in onInit
    private ColorChannel[] channels;

    // -------------------------------------------------------------------------
    // KVisionProcessor hooks
    // -------------------------------------------------------------------------

    @Override
    protected void onInit(int frameWidth, int frameHeight) {
        widthScaleFactor  = (double) frameWidth  / PROCESSING_WIDTH;
        heightScaleFactor = (double) frameHeight / PROCESSING_HEIGHT;

        channels = defineChannels();

        downsampledFrame       = new Mat();
        blurredFrame           = new Mat();
        hsvFrame               = new Mat();
        morphologyBuffer       = new Mat();
        reusableContourBuffer  = new MatOfPoint2f();
        noiseRemovalKernel  = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(3, 3));
        holeFillingKernel   = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(7, 7));
        contourHierarchy    = new Mat();

        for (ColorChannel channel : channels) {
            channel.mask      = new Mat();
            channel.boxPaint  = makeStrokePaint(channel.overlayColor);
            channel.textPaint = makeMonospaceTextPaint(channel.overlayColor);
        }
        crosshairPaint = makeStrokePaint(Color.WHITE);
        crosshairPaint.setStrokeWidth(CROSSHAIR_STROKE_DP);
    }

    @Override
    protected List<DetectedBlob> detect(Mat frame) {
        Imgproc.resize(frame, downsampledFrame,
                new Size(PROCESSING_WIDTH, PROCESSING_HEIGHT), 0, 0, Imgproc.INTER_LINEAR);
        Imgproc.GaussianBlur(downsampledFrame, blurredFrame,
                new Size(gaussianKernelSize, gaussianKernelSize), 0);
        Imgproc.cvtColor(blurredFrame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        List<DetectedBlob> allBlobs = new ArrayList<>();
        for (ColorChannel channel : channels) {
            Core.inRange(hsvFrame, channel.hsvLowerBound, channel.hsvUpperBound, channel.mask);
            allBlobs.addAll(extractBlobsFromMask(channel.mask, channel.label));
        }

        allBlobs.sort((first, second) -> Double.compare(second.area, first.area));
        return allBlobs;
    }

    @Override
    protected void annotate(Canvas canvas, List<DetectedBlob> blobs, DrawContext drawContext) {
        for (DetectedBlob blob : blobs) {
            ColorChannel channel = channelFor(blob.colorLabel);
            if (channel == null) continue;

            channel.boxPaint.setStrokeWidth(STROKE_WIDTH_DP * drawContext.screenDensityScale);
            channel.textPaint.setTextSize(TEXT_SIZE_DP * drawContext.screenDensityScale);

            float boxLeft   = (float) blob.boundingBox.x * drawContext.bitmapToCanvasScale;
            float boxTop    = (float) blob.boundingBox.y * drawContext.bitmapToCanvasScale;
            float boxRight  = (float)(blob.boundingBox.x + blob.boundingBox.width)  * drawContext.bitmapToCanvasScale;
            float boxBottom = (float)(blob.boundingBox.y + blob.boundingBox.height) * drawContext.bitmapToCanvasScale;
            canvas.drawRect(boxLeft, boxTop, boxRight, boxBottom, channel.boxPaint);
            canvas.drawText(blob.label, boxLeft,
                    boxTop - LABEL_OFFSET_DP * drawContext.screenDensityScale, channel.textPaint);

            float centerX = (float) blob.center.getX() * drawContext.bitmapToCanvasScale;
            float centerY = (float) blob.center.getY() * drawContext.bitmapToCanvasScale;
            canvas.drawLine(centerX - drawContext.crosshairArmLength, centerY,
                            centerX + drawContext.crosshairArmLength, centerY, crosshairPaint);
            canvas.drawLine(centerX, centerY - drawContext.crosshairArmLength,
                            centerX, centerY + drawContext.crosshairArmLength, crosshairPaint);
        }
    }

    // -------------------------------------------------------------------------
    // Public accessors — called from robot/action thread
    // -------------------------------------------------------------------------

    /** Largest blob across all channels. Null if nothing detected. */
    public DetectedBlob getLargestBlob() {
        List<DetectedBlob> snapshot = getLatestResult();
        return (snapshot == null || snapshot.isEmpty()) ? null : snapshot.get(0);
    }

    /** Largest blob matching the given channel label. Null if not detected. */
    public DetectedBlob getLargestBlobByLabel(String colorLabel) {
        List<DetectedBlob> snapshot = getLatestResult();
        if (snapshot == null) return null;
        for (DetectedBlob blob : snapshot) {
            if (colorLabel.equals(blob.colorLabel)) return blob;
        }
        return null;
    }

    public boolean hasBlobWithLabel(String colorLabel) {
        return getLargestBlobByLabel(colorLabel) != null;
    }

    // -------------------------------------------------------------------------
    // Internal helpers
    // -------------------------------------------------------------------------

    private List<DetectedBlob> extractBlobsFromMask(Mat colorMask, String colorLabel) {
        Imgproc.morphologyEx(colorMask, morphologyBuffer, Imgproc.MORPH_OPEN, noiseRemovalKernel);
        Imgproc.morphologyEx(morphologyBuffer, morphologyBuffer, Imgproc.MORPH_CLOSE, holeFillingKernel);

        reusableContourList.clear();
        Imgproc.findContours(morphologyBuffer, reusableContourList, contourHierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<DetectedBlob> blobs = new ArrayList<>();
        for (MatOfPoint contour : reusableContourList) {
            double area = Imgproc.contourArea(contour);
            if (area < minContourArea || area > maxContourArea) {
                contour.release();
                continue;
            }

            reusableContourBuffer.fromArray(contour.toArray());
            double circularity = computeCircularity(area, Imgproc.arcLength(reusableContourBuffer, true));
            if (circularity < minCircularity) {
                contour.release();
                continue;
            }

            blobs.add(new DetectedBlob(
                scaleRectToFullResolution(Imgproc.boundingRect(contour)),
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
