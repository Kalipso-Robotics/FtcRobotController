package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

import java.util.Locale;

/**
 * A single object detection result from a KRoboflowDetector.
 *
 * Coordinates are in the original full-resolution camera frame (pixels).
 * The bounding box follows screen conventions: (0,0) is top-left.
 */
public class RoboflowRecognition {

    /** Class label from the model (e.g. "YellowRect", "PurpleBall"). */
    public final String label;

    /** Detection confidence in [0.0, 1.0]. */
    public final float confidence;

    /** Bounding box edges in full-resolution frame pixels. */
    public final float left;
    public final float top;
    public final float right;
    public final float bottom;

    /** Center of the bounding box. */
    public final Point center;

    /** Pre-formatted label string ready for telemetry, e.g. "YellowRect 87%". */
    public final String formattedLabel;

    public RoboflowRecognition(String label, float confidence,
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

    @Override
    public String toString() {
        return String.format(Locale.US,
                "RoboflowRecognition{label='%s', confidence=%.2f, box=[%.0f,%.0f,%.0f,%.0f]}",
                label, confidence, left, top, right, bottom);
    }
}