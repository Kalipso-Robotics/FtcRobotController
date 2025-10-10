package com.kalipsorobotics.utilities;

import static com.kalipsorobotics.modules.MotifColors.NONE;

import com.kalipsorobotics.modules.MotifColors;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import java.util.HashMap;

public class KColorDetection {

    private static final String CALIBRATION_FILENAME = "color_calibration.csv";


    public static MotifColors detectColor(String sensorName, RevColorSensorV3 revColor, OpModeUtilities opModeUtilities) {
        HashMap<MotifColors, HSV> calibration = loadCalibration(sensorName, opModeUtilities);
        return detectColor(calibration, revColor);
    }

    public static MotifColors detectColor(HashMap<MotifColors, HSV> revValues, RevColorSensorV3 revColor) {
        int red   = revColor.red();
        int green = revColor.green();
        int blue  = revColor.blue();

        float[] testHSV = new float[3];
        android.graphics.Color.RGBToHSV(red, green, blue, testHSV);
        HSV testValue = new HSV(testHSV[0], testHSV[1], testHSV[2]);

        double minDistance = Double.MAX_VALUE;
        MotifColors closestColor = NONE;

        for (MotifColors color : revValues.keySet()) {
            HSV calibratedValue = revValues.get(color);

            double distance = calculateHSVDistance(testValue, calibratedValue);
            if (distance < minDistance) {
                minDistance = distance;
                closestColor = color;
            }
        }

        return closestColor;
    }


    public static MotifColors detectColor(HashMap<MotifColors, HSV> revValues, HSV testValue) {

        double bestDistance = Double.MAX_VALUE;
        MotifColors closestColor = NONE;

        for (MotifColors color : revValues.keySet()) {
            HSV calibratedValue = revValues.get(color);

            double distance = calculateHSVDistance(testValue, calibratedValue);
            if (distance < bestDistance) {
                bestDistance = distance;
                closestColor = color;
            }
        }

        return closestColor;
    }

    private static double calculateHSVDistance(HSV test, HSV ref) {
        // Hue: circular shortest distance in degrees -> [0,180], then normalize
        double h = Math.abs(test.getHue() - ref.getHue());
        if (h > 180) h = 360 - h;
        h /= 180.0;

        double s = test.getSaturation() - ref.getSaturation();

        // final calculation (can be tuned) currently uses simple distance formula and equal weight between h and s
        return Math.sqrt(h*h + s*s);
    }

    /*
    untested some code that gpt john suggested based off the graphs of color distibution. The idea is to weight hue more that saturation

    private static double calculateHSVDistance(HSV test, HSV ref) {
    double h = Math.abs(test.getHue() - ref.getHue());
    if (h > 180) h = 360 - h;      // circular hue diff
    h /= 60.0;                     // scale hue separation

    double s = (test.getSaturation() - ref.getSaturation()) / 0.25;

    // weights: emphasize Hue strongly, Saturation moderately
    double wH = 1.0, wS = 0.6;

    return Math.sqrt(wH*h*h + wS*s*s);
}
    */

    public static HashMap<MotifColors, HSV> loadCalibration(String sensorName, OpModeUtilities opModeUtilities) {
        HashMap<MotifColors, HSV> calibrationMap = new HashMap<>();

        try {
            KFileReader reader = new KFileReader(CALIBRATION_FILENAME, opModeUtilities);
            String line;

            while ((line = reader.readLine()) != null) {
                if (line.startsWith("Sensor")) {
                    continue;
                }

                String[] values = line.split(",");
                if (values.length < 5) {
                    continue;
                }

                String sensor = values[0];
                if (!sensor.equals(sensorName)) {
                    continue;
                }

                MotifColors color = MotifColors.valueOf(values[1]);
                float hue = Float.parseFloat(values[2]);
                float sat = Float.parseFloat(values[3]);
                float val = Float.parseFloat(values[4]);

                HSV hsv = new HSV(hue, sat, val);
                calibrationMap.put(color, hsv);
            }

            reader.close();
            KLog.d("ColorCalibrationDetection", "Loaded calibration for " + sensorName);
        } catch (Exception e) {
            KLog.d("ColorCalibrationDetection", "Failed to load calibration for " + sensorName + ": " + e.getMessage());
        }

        return calibrationMap;
    }
}
