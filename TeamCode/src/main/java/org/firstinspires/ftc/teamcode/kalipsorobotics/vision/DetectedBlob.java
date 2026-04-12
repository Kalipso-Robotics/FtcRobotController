package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.opencv.core.Rect;

import java.util.Locale;

import androidx.annotation.NonNull;

/**
 * A single color blob detected by KColorBlobProcessor.
 * Shared across all color-blob-based processors so downstream code
 * (actions, IK, ray tracer) never needs to import processor-specific types.
 */
public class DetectedBlob {

    /** Bounding box in full camera resolution (e.g. 640x480) pixels. */
    public final Rect boundingBox;

    /** Blob center in full camera resolution pixels. Uses the team's Point class. */
    public final Point center;

    /** Contour area measured in the downsampled processing frame (320x240). */
    public final double area;

    /** 0.0–1.0. 1.0 = perfect circle. Proxy for detection confidence. */
    public final double circularity;

    /** Human-readable label matching the ColorChannel name (e.g. "Purple", "Yellow"). */
    public final String colorLabel;

    /** Pre-formatted telemetry string — avoids String.format in draw loops. */
    final String label;

    DetectedBlob(Rect boundingBox, double area, double circularity, String colorLabel) {
        this.boundingBox = boundingBox;
        this.center = new Point(
            boundingBox.x + boundingBox.width  / 2.0,
            boundingBox.y + boundingBox.height / 2.0
        );
        this.area = area;
        this.circularity = circularity;
        this.colorLabel = colorLabel;
        this.label = String.format(Locale.US, "%s (%.0f,%.0f) A:%.0f C:%.2f",
            colorLabel, center.getX(), center.getY(), area, circularity);
    }

    @Override
    @NonNull
    public String toString() { return label; }
}
