package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

import java.util.Locale;

/**
 * A single object recognition produced by any KVisionProcessor — TFLite, color blob, AprilTag, etc.
 *
 * Coordinates are in the original full-resolution camera frame (pixels).
 * The bounding box follows screen conventions: (0,0) is top-left.
 *
 * Subclass this when a processor produces detection-specific extras
 * (e.g. DetectedBlob adds contour area and circularity).
 */
public class VisionRecognition {

    /** Label identifying what was detected (e.g. "Purple", "Green", "Artifact"). */
    public final String label;

    /** Detection confidence in [0.0, 1.0]. Color blobs use circularity as a proxy. */
    public final float confidence;

    /** Bounding box edges in full-resolution frame pixels. */
    public final float left;
    public final float top;
    public final float right;
    public final float bottom;

    /** Center of the bounding box. */
    public final Point center;

    /** Pre-formatted label string ready for telemetry, e.g. "Purple 87%". */
    public final String formattedLabel;

    public VisionRecognition(String label, float confidence,
                             float left, float top, float right, float bottom) {
        this.label      = label;
        this.confidence = confidence;
        this.left       = left;
        this.top        = top;
        this.right      = right;
        this.bottom     = bottom;
        this.center     = new Point((left + right) / 2.0, (top + bottom) / 2.0);
        this.formattedLabel = String.format(Locale.US, "%s %.0f%%", label, confidence * 100);
    }

    public float getWidth()  { return right - left; }
    public float getHeight() { return bottom - top; }

    /** Bottom-middle pixel of the bounding box — used by CameraIntrinsics for floor projection. */
    public Point getBottomMiddlePixel() {
        return new Point((left + right) / 2.0, bottom);
    }

    /** Detection area. Default: bounding box area. Subclasses can override (e.g. contour area). */
    public double getArea() {
        return getWidth() * getHeight();
    }

    /** Circularity [0, 1]. Default 0 — only meaningful for contour-based detections. */
    public double getCircularity() { return 0.0; }

    @Override
    public String toString() {
        return String.format(Locale.US,
                "KVisionRecognition{label='%s', confidence=%.2f, box=[%.0f,%.0f,%.0f,%.0f]}",
                label, confidence, left, top, right, bottom);
    }
}
