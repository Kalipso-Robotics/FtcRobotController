package org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblobbing;

import android.graphics.Color;

import org.opencv.core.Scalar;

public class PollenColorBlobDetectionProcessor extends KColorBlobProcessor{

    public static final String YELLOW = "Yellow";
    static final Scalar YELLOW_HSV_LOWER = new Scalar(22,81,79);
    static final Scalar YELLOW_HSV_UPPER = new Scalar(33,255,255);





    @Override
    protected ColorChannel[] defineChannels() {
        return new ColorChannel[]{
                new ColorChannel(YELLOW_HSV_LOWER, YELLOW_HSV_UPPER,YELLOW, Color.rgb(255,255,0))
        };
    }
    public DetectedBlob getLargestYellowBlob(){return getLargestBlobByLabel(YELLOW);}
    public boolean hasYellowBlob(){return hasBlobWithLabel(YELLOW);}


}
