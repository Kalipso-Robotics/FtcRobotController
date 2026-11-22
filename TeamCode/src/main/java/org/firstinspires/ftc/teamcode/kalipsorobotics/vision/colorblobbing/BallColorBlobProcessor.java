package org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblobbing;

import android.graphics.Color;

import org.opencv.core.Scalar;

/**
 * Pollen (yellow) and nectar (red, blue) by color, the no-model alternative to TFLitePollenNectarDetector.
 * Labels match Raytracer.typeFor ("Yellow", "Red", "Blue"), so the Raytracer turns blobs into balls unchanged.
 * Red wraps around hue 0, so it is two channels with the same label.
 */
public class BallColorBlobProcessor extends KColorBlobProcessor {
    public static final String YELLOW = PollenColorBlobDetectionProcessor.YELLOW;
    public static final String RED = "Red";
    public static final String BLUE = "Blue";

    // TODO-measure: red and blue are the textbook ranges from KColorBlobProcessor's HSV reference, not tuned on the
    // field. Tune with pollen_tuner.py / PollenNectarLive (S_min > 50, V_min > 40 keeps the background out).
    static final Scalar RED_LOW_HSV_LOWER = new Scalar(0, 100, 60);
    static final Scalar RED_LOW_HSV_UPPER = new Scalar(10, 255, 255);
    static final Scalar RED_HIGH_HSV_LOWER = new Scalar(170, 100, 60);
    static final Scalar RED_HIGH_HSV_UPPER = new Scalar(180, 255, 255);
    static final Scalar BLUE_HSV_LOWER = new Scalar(100, 100, 50);
    static final Scalar BLUE_HSV_UPPER = new Scalar(130, 255, 255);

    {
        objectDiameterMM = PollenColorBlobDetectionProcessor.POLLEN_DIAMETER_MM; // pollen and nectar are both 2.8 in
    }

    @Override
    protected ColorChannel[] defineChannels() {
        return new ColorChannel[]{
                new ColorChannel(PollenColorBlobDetectionProcessor.YELLOW_HSV_LOWER, PollenColorBlobDetectionProcessor.YELLOW_HSV_UPPER,
                        YELLOW, Color.rgb(255, 255, 0)),
                new ColorChannel(RED_LOW_HSV_LOWER, RED_LOW_HSV_UPPER, RED, Color.rgb(255, 0, 0)),
                new ColorChannel(RED_HIGH_HSV_LOWER, RED_HIGH_HSV_UPPER, RED, Color.rgb(255, 0, 0)),
                new ColorChannel(BLUE_HSV_LOWER, BLUE_HSV_UPPER, BLUE, Color.rgb(0, 0, 255)),
        };
    }
}
