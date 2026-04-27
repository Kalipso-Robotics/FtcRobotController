package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import android.graphics.Color;

import org.opencv.core.Scalar;

/**
 * Detects purple and green artifacts for FTC DECODE.
 *
 * All detection logic, drawing, and threading live in KColorBlobProcessor.
 * This class only defines the two colors to look for.
 *
 * To add a color next season, add a new ColorChannel here — nothing else changes.
 */
public class ArtifactDetectionProcessor extends KColorBlobProcessor {

    public static final String PURPLE = "Purple";
    public static final String GREEN  = "Green";

    private static final Scalar PURPLE_HSV_LOWER = new Scalar(117, 58, 54);
    private static final Scalar PURPLE_HSV_UPPER = new Scalar(180, 255, 255);
    private static final Scalar GREEN_HSV_LOWER  = new Scalar(68, 70, 22);
    private static final Scalar GREEN_HSV_UPPER  = new Scalar(92, 255, 255);

    @Override
    protected ColorChannel[] defineChannels() {
        return new ColorChannel[] {
            new ColorChannel(PURPLE_HSV_LOWER, PURPLE_HSV_UPPER, PURPLE, Color.rgb(180, 0, 255)),
            new ColorChannel(GREEN_HSV_LOWER,  GREEN_HSV_UPPER,  GREEN,  Color.rgb(0, 220, 60))
        };
    }

    // Named accessors so callers never write string literals
    public DetectedBlob getLargestPurpleBlob() { return getLargestBlobByLabel(PURPLE);}
    public DetectedBlob getLargestGreenBlob() { return getLargestBlobByLabel(GREEN);}
    public boolean hasPurpleBlob() { return hasBlobWithLabel(PURPLE);}
    public boolean hasGreenBlob() { return hasBlobWithLabel(GREEN);}
}
