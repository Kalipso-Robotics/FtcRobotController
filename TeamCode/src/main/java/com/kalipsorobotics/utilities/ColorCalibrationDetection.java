package com.kalipsorobotics.utilities;

import static com.kalipsorobotics.utilities.KColor.Color.NONE;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import java.util.HashMap;

public class ColorCalibrationDetection {

    private static final String CALIBRATION_FILENAME = "color_calibration.csv";

    /**
     * Detect color using auto-loaded calibration data
     * <p>
     * Auto-loads calibration from file and detects color. Calibration must be saved first using ColorCalibration OpMode.
     * <p>
     * Example usage:
     * <pre>
     * RevColorSensorV3 frontSensor = hardwareMap.get(RevColorSensorV3.class, "revColor1");
     * KColor.Color detected = ColorCalibrationDetection.detectColor("Front", frontSensor, opModeUtilities);
     * </pre>
     *
     * @param sensorName Sensor identifier - MUST match exactly: "Front", "BLeft", or "BRight" (case-sensitive)
     *                   These correspond to revColor1, revColor2, revColor3 respectively
     * @param revColor The RevColorSensorV3 sensor instance to read from
     * @param opModeUtilities OpModeUtilities instance for file access
     * @return Detected color (PURPLE, GREEN, or NONE)
     */
    public static KColor.Color detectColor(String sensorName, RevColorSensorV3 revColor, OpModeUtilities opModeUtilities) {
        HashMap<KColor.Color, HSV> calibration = loadCalibration(sensorName, opModeUtilities);
        return detectColor(calibration, revColor);
    }

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

    private static double calculateHSVDistance(HSV test, HSV ref) {
        // Hue: circular shortest distance in degrees -> [0,180], then normalize
        double h = Math.abs(test.getHue() - ref.getHue());
        if (h > 180) h = 360 - h;
        h /= 180.0;

        double s = test.getSaturation() - ref.getSaturation();

//        // Value can be >1 because inputs to RGBToHSV weren't 0..255.
//        // Normalize by a soft scale so it can't swamp the metric.
//        double valueScale = 2.0; // TUNE: 1.0–4.0; start at 2
//        double v = (test.getValue() - ref.getValue()) / valueScale;

        // weights: emphasize hue to split green vs purple cleanly
//        double wH = 4.0, wS = 1.0, wV = 0.25;
        return Math.sqrt(h*h + s*s);
    }

    /**
     * Load calibration data for a specific sensor from saved file
     * <p>
     * Loads previously saved calibration from color_calibration.csv in OdometryLog directory.
     * Use this if you want to cache calibration data instead of loading it every detection call.
     * <p>
     * Example usage:
     * <pre>
     * // Load once during init
     * HashMap&lt;KColor.Color, HSV&gt; frontCal = ColorCalibrationDetection.loadCalibration("Front", opModeUtilities);
     * // Use multiple times in loop
     * KColor.Color detected = ColorCalibrationDetection.detectColor(frontCal, frontSensor);
     * </pre>
     *
     * @param sensorName Sensor identifier - MUST be exactly: "Front", "BLeft", or "BRight" (case-sensitive)
     * @param opModeUtilities OpModeUtilities instance for file access
     * @return HashMap with calibrated HSV values for PURPLE, GREEN, and NONE
     */
    public static HashMap<KColor.Color, HSV> loadCalibration(String sensorName, OpModeUtilities opModeUtilities) {
        HashMap<KColor.Color, HSV> calibrationMap = new HashMap<>();

        try {
            KFileReader reader = new KFileReader(CALIBRATION_FILENAME, opModeUtilities);
            String line;

            while ((line = reader.readLine()) != null) {
                if (line.startsWith("Sensor")) {
                    continue;
                }

                String[] values = line.split(",");
                if (values.length < 5) continue;

                String sensor = values[0];
                if (!sensor.equals(sensorName)) continue;

                KColor.Color color = KColor.Color.valueOf(values[1]);
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
