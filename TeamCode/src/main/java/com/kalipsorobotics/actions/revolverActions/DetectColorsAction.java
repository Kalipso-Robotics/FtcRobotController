package com.kalipsorobotics.actions.revolverActions;

import static com.kalipsorobotics.modules.MotifColor.NONE;

import android.graphics.Color;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import com.kalipsorobotics.modules.ColorSensorPosition;
import com.kalipsorobotics.modules.MotifColor;
import com.kalipsorobotics.modules.TripleColorSensor;
import com.kalipsorobotics.utilities.HSV;
import com.kalipsorobotics.utilities.KFileReader;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import java.util.HashMap;

public class DetectColorsAction extends Action {

    public static final String CALIBRATION_FILENAME = "color_calibration.csv";

    private OpModeUtilities opModeUtilities;
    TripleColorSensor colorSensors;
    static HashMap<MotifColor, HSV> calibratedFront = null;
    static HashMap<MotifColor, HSV> calibratedBLeft = null;
    static HashMap<MotifColor, HSV> calibratedBRight = null;
    HashMap<ColorSensorPosition, HSV> rawValues = new HashMap<>();
    HashMap<ColorSensorPosition, MotifColor> calculatedColorValue = new HashMap<>();
    public DetectColorsAction(TripleColorSensor colorSensors, OpModeUtilities opModeUtilities) {
        this.colorSensors = colorSensors;
        this.opModeUtilities = opModeUtilities;
        this.dependentActions.add(new DoneStateAction());
        if (calibratedFront == null || calibratedBLeft == null || calibratedBRight == null) {
            loadCalibrationData();
        }
    }

    @Override
    public void update() {
        if (!hasStarted) {
            hasStarted = true;
        }
        for (ColorSensorPosition position : colorSensors.getSensors().keySet()) {
            RevColorSensorV3 sensor = colorSensors.getSensors().get(position);

            int red = sensor.red();
            int green = sensor.green();
            int blue = sensor.blue();

            float[] testHSV = new float[3];
            Color.RGBToHSV(red, green, blue, testHSV);
            HSV rawValue = new HSV(testHSV[0], testHSV[1], testHSV[2]);
            rawValues.put(position, rawValue);

        }
        calculatedColorValue.put(ColorSensorPosition.FRONT, calculateColor(calibratedFront, rawValues.get(ColorSensorPosition.FRONT)));
        calculatedColorValue.put(ColorSensorPosition.BRIGHT, calculateColor(calibratedBRight, rawValues.get(ColorSensorPosition.BRIGHT)));
        calculatedColorValue.put(ColorSensorPosition.BLEFT, calculateColor(calibratedBLeft, rawValues.get(ColorSensorPosition.BLEFT)));

        isDone = true;
    }


    private void loadCalibrationData() {
        try {
            calibratedFront = new HashMap<>();
            calibratedBLeft = new HashMap<>();
            calibratedBRight = new HashMap<>();

            KFileReader reader = new KFileReader(CALIBRATION_FILENAME, opModeUtilities);
            String line;

            while ((line = reader.readLine()) != null) {
                if (line.startsWith("Sensor")) {
                    continue;
                }

                String[] values = line.split(",");
                if (values.length < 5) continue;

                String sensor = values[0];
                MotifColor color = MotifColor.valueOf(values[1]);
                float hue = Float.parseFloat(values[2]);
                float sat = Float.parseFloat(values[3]);
                float val = Float.parseFloat(values[4]);

                HSV hsv = new HSV(hue, sat, val);

                if (sensor.equals(ColorSensorPosition.FRONT.toString())) {
                    calibratedFront.put(color, hsv);
                } else if (sensor.equals(ColorSensorPosition.BLEFT.toString())) {
                    calibratedBLeft.put(color, hsv);
                } else if (sensor.equals(ColorSensorPosition.BRIGHT.toString())) {
                    calibratedBRight.put(color, hsv);
                }
            }

            reader.close();
            KLog.d("ColorCalibration", "Calibration data loaded successfully");
        } catch (Exception e) {
            KLog.d("ColorCalibration", "No existing calibration data found or failed to load: " + e.getMessage());
        }
    }

//    public MotifColors detectColor(String sensorName, RevColorSensorV3 revColor, OpModeUtilities opModeUtilities) {
//        loadCalibrationData();
//        return detectColor(calibration, revColor);
//    }

    private MotifColor calculateColor(HashMap<MotifColor, HSV> calibrationMap, HSV testValue) {
        if (calibrationMap == null || calibrationMap.isEmpty() || testValue == null) {
            return NONE;
        }

        double minDistance = Double.MAX_VALUE;
        MotifColor closestColor = NONE;

        for (MotifColor color : calibrationMap.keySet()) {
            HSV calibratedValue = calibrationMap.get(color);

            if (calibratedValue == null) continue;

            double distance = calculateHSVDistance(testValue, calibratedValue);
            if (distance < minDistance) {
                minDistance = distance;
                closestColor = color;
            }
        }

        return closestColor;
    }

    public MotifColor[] transformColorSetToTray(MotifColor[] colorSet, int currentRevolverIndex) {
        MotifColor[] transformedColorSet = new MotifColor[3];

        //transformed color set holds color of ball in each revolver tray index,
        //rather than color above each color sensor index.
        if (currentRevolverIndex == 0) {
            transformedColorSet = colorSet;
        } else if (currentRevolverIndex == 1) {
            transformedColorSet[0] = colorSet[2];
            transformedColorSet[1] =  colorSet[0];
            transformedColorSet[2] = colorSet[1];
        } else if (currentRevolverIndex == 2) {
            transformedColorSet[0] = colorSet[1];
            transformedColorSet[1] =  colorSet[2];
            transformedColorSet[2] = colorSet[0];
        }

        return transformedColorSet;
    }

    public MotifColor getFrontColor() {
        return calculatedColorValue.get(ColorSensorPosition.FRONT);
    }
    public MotifColor getBLeftColor() {
        return calculatedColorValue.get(ColorSensorPosition.BLEFT);
    }
    public MotifColor getBrightColor() {
        return calculatedColorValue.get(ColorSensorPosition.BRIGHT);
    }
    public HashMap<ColorSensorPosition, MotifColor> getCalculatedColorValue() {
        return calculatedColorValue;
    }
    public HashMap<ColorSensorPosition, HSV> getRawValues() {
        return rawValues;
    }

    public MotifColor[] getColorSet(){
        MotifColor[] colorSet = new MotifColor[3];
        colorSet[0] = getFrontColor();
        colorSet[1] = getBrightColor();
        colorSet[2] = getBLeftColor();
        return colorSet;
    }

    public static void clearCalibratedMap() {
        calibratedFront = null;
        calibratedBLeft = null;
        calibratedBRight = null;
    }

    private double calculateHSVDistance(HSV test, HSV ref) {
        double h = Math.abs(test.getHue() - ref.getHue());
        if (h > 180) h = 360 - h;      // circular hue diff
        h /= 60.0;                     // scale hue separation

        double s = (test.getSaturation() - ref.getSaturation()) / 0.25;

        // weights: emphasize Hue strongly, Saturation moderately
        double wH = 1.0, wS = 0.6;

        return Math.sqrt(wH * h * h + wS * s * s);
    }
}
