package com.kalipsorobotics.utilities;

import static com.kalipsorobotics.utilities.KColor.Color.NONE;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import java.util.HashMap;

public class ColorCalibrationDetection {

    public static KColor.Color detectColor(HashMap<KColor.Color, HSV> revValues, RevColorSensorV3 revColor) {
        int red   = revColor.red();
        int green = revColor.green();
        int blue  = revColor.blue();

        float[] testHSV = new float[3];
        android.graphics.Color.RGBToHSV(red, green, blue, testHSV);
        HSV testValue = new HSV(testHSV[0], testHSV[1], testHSV[2]);

        double minDistance = Double.MAX_VALUE;
        KColor.Color closestColor = NONE;

        for (KColor.Color color : revValues.keySet()) {
            HSV calibratedValue = revValues.get(color);

            double distance = calculateHSVDistance(testValue, calibratedValue);
            if (distance < minDistance) {
                minDistance = distance;
                closestColor = color;
            }
        }

        return closestColor;
    }

    public static KColor.Color detectColor(HashMap<KColor.Color, HSV> revValues, HSV testValue) {

        double bestDistance = Double.MAX_VALUE;
        KColor.Color closestColor = NONE;

        for (KColor.Color color : revValues.keySet()) {
            HSV calibratedValue = revValues.get(color);

            double distance = calculateHSVDistance(testValue, calibratedValue);
            if (distance < bestDistance) {
                bestDistance = distance;
                closestColor = color;
            }
        }

        return closestColor;
    }

    private static double calculateHSVDistance(HSV hsv1, HSV hsv2) {
        double hueDist = Math.abs(hsv1.getHue() - hsv2.getHue());
        if (hueDist > 180) {
            hueDist = 360 - hueDist;
        }
        hueDist /= 180;
        hueDist = hueDist*hueDist;
        // circular hue wrapping 360 degrees wraps to 180, 360 and 10 are not 350 degrees away but rather 20

        double satDist = Math.pow((hsv1.getSaturation() - hsv2.getSaturation()), 2);
        double valDist = Math.pow(hsv1.getValue() - hsv2.getValue(),2);

        return Math.sqrt(hueDist + satDist  + valDist);
    }

}
