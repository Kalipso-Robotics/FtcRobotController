package com.kalipsorobotics.utilities;

public class HSV {

    private float hue;
    private float saturation;
    private float value;

    public HSV(float hue, float saturation, float value) {
        this.hue = hue;
        this.saturation = saturation;
        this.value = value;
    }

    public HSV(int red, int green, int blue) {

        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV(red, green, blue, hsv);

        this.hue = hsv[0];        // 0–360 degrees
        this.saturation = hsv[1]; // 0–1
        this.value = hsv[2];
    }

    public float getHue(){
        return hue;
    }
    public float getSaturation() {
        return saturation;
    }
    public float getValue() {
        return value;
    }

    public void avgHSV(float newHue, float newSaturation, float newValue) {
        hue = (hue+newHue)/2;
        saturation = (saturation + newSaturation)/2;
        value = (value + newValue)/2;
    }

    @Override
    public String toString() {
        return "HSV{" +
                "hue=" + hue +
                ", saturation=" + saturation +
                ", value=" + value +
                '}';
    }
}
