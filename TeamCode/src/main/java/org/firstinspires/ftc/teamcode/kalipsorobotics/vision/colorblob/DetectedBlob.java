package org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblob;

import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionRecognition;
import org.opencv.core.Rect;

import java.util.Locale;

/**
 * A single color blob detected by KColorBlobProcessor.
 *
 * Adds contour-area and circularity on top of the base KVisionRecognition.
 * The `label` field holds the color name ("Purple", "Green", etc.).
 */
public class DetectedBlob extends VisionRecognition {

    /** Bounding box in full camera resolution (e.g. 640x480) pixels. */
    public final Rect boundingBox;

    /** Contour area measured in the downsampled processing frame (320x240). */
    public final double contourArea;

    /** 0.0–1.0. 1.0 = perfect circle. Used as the parent confidence value. */
    public final double circularity;

    DetectedBlob(Rect boundingBox, double contourArea, double circularity, String colorLabel) {
        super(colorLabel, (float) circularity,
                boundingBox.x,
                boundingBox.y,
                boundingBox.x + boundingBox.width,
                boundingBox.y + boundingBox.height);
        this.boundingBox = boundingBox;
        this.contourArea = contourArea;
        this.circularity = circularity;
    }

    /** Returns the contour area (more accurate than the bounding-box area). */
    @Override
    public double getArea() { return contourArea; }

    @Override
    public double getCircularity() { return circularity; }

    @Override
    public String toString() {
        return String.format(Locale.US, "%s (%.0f,%.0f) A:%.0f C:%.2f",
                label, center.getX(), center.getY(), contourArea, circularity);
    }
}
