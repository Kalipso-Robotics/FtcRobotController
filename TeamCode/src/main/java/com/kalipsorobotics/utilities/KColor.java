package com.kalipsorobotics.utilities;

import com.qualcomm.hardware.rev.RevColorSensorV3;

public class KColor {
    public enum Color {
        PURPLE, GREEN, NONE
    }

    int red, green, blue;
    public static Color classify(RevColorSensorV3 revColor) {
        int red = revColor.red();
        int green = revColor.green();
        int blue = revColor.blue();

        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV(red, green, blue, hsv);

        float hue = hsv[0];        // 0–360 degrees
        float saturation = hsv[1]; // 0–1
        float value = hsv[2];      // 0–1


        // NONE
        if (saturation < 0.35 || value < 0.20 || value > 5.5) return KColor.Color.NONE;

        // PURPLE
        if (hue >= 200 && hue <= 240 && saturation >= 0.55 && value >= 0.20 && value <= 5.5) {
            return KColor.Color.PURPLE;
        }
        // GREEN
        if (hue >= 60 && hue <= 150 && saturation >= 0.38 && value >= 0.20 && value <= 5.5) {
            return KColor.Color.GREEN;
        }

        return KColor.Color.NONE;
    }

    public static String printColor(RevColorSensorV3 revColor) {
        return "Red: " + revColor.red() + " Green: " + revColor.green() + " Blue: " + revColor.blue();
    }

    public static String printHSV(RevColorSensorV3 revColor) {
        int red = revColor.red();
        int green = revColor.green();
        int blue = revColor.blue();

        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV(red, green, blue, hsv);

        float hue = hsv[0];        // 0–360 degrees
        float saturation = hsv[1]; // 0–1
        float value = hsv[2];      // 0–1

        return "Hue: " + hue + " Saturation: " +saturation + " Value: " + value;
    }
    public KColor(int red, int green, int blue) {
        this.red = red;
        this.green = green;
        this.blue = blue;
    }
    public KColor() {
    }
}
