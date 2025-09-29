package com.kalipsorobotics.utilities;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;

public class KColor {
    public enum Color {
        RED, YELLOW, GREEN, BLUE, PURPLE, NONE
    }
    int red, green, blue;
    public int getGreen() {
        return green;
    }
    public void setGreen(int green) {
        this.green = green;
    }

    public int getRed() {
        return red;
    }

    public void setRed(int red) {
        this.red = red;
    }
    public int getBlue() {
        return blue;
    }
    public void setBlue(int blue) {
        this.blue = blue;
    }

    public static Color classify(RevColorSensorV3 revColor) {
        int red = revColor.red();
        int green = revColor.green();
        int blue = revColor.blue();

        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV(red, green, blue, hsv);

        float hue = hsv[0];        // 0–360 degrees
        float saturation = hsv[1]; // 0–1
        float value = hsv[2];      // 0–1

        if (hue > 250 && hue < 300 && saturation > 0.5 && value > 0.2) {
            return KColor.Color.PURPLE;
        }
        else if (hue > 80 && hue < 160 && saturation > 0.5 && value > 0.2) {
            return KColor.Color.GREEN;
        }
        else {
            return KColor.Color.NONE;
        }
    }
    public KColor(int red, int green, int blue) {
        this.red = red;
        this.green = green;
        this.blue = blue;
    }
    public KColor() {
    }
}
