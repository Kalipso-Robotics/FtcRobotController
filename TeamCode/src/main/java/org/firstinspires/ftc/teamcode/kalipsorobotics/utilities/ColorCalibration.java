package org.firstinspires.ftc.teamcode.kalipsorobotics.utilities;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.actions.revolverActions.DetectColorsAction.CALIBRATION_FILENAME;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.MotifColor.PURPLE;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.MotifColor.GREEN;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.MotifColor.NONE;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.revolverActions.DetectColorsAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.ColorSensorPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.MotifColor;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.TripleColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;

@Disabled
public class ColorCalibration extends LinearOpMode {
    MotifColor currentColor;
    KGamePad kPad;
    KFileWriter kFile;
    TripleColorSensor colorSensors;
    OpModeUtilities opModeUtilities;
    LinearOpMode linearOpMode;
    DetectColorsAction detectColorsAction;
    HashMap<MotifColor, HSV> calibratedFront = null;
    HashMap<MotifColor, HSV> calibratedBLeft = null;
    HashMap<MotifColor, HSV> calibratedBRight = null;
    HashMap<ColorSensorPosition, HSV> rawValues;

    // Uncomment to collect raw data points for analysis
    private KFileWriter rawDataFile;

    @Override
    public void runOpMode() throws InterruptedException {

        opModeUtilities = new OpModeUtilities(hardwareMap, linearOpMode, telemetry);

        colorSensors = new TripleColorSensor(opModeUtilities);
        kPad = new KGamePad(gamepad1);

        rawValues = new HashMap<>();
        calibratedFront = new HashMap<>();
        calibratedBRight = new HashMap<>();
        calibratedBLeft= new HashMap<>();

        detectColorsAction = new DetectColorsAction(colorSensors, opModeUtilities);

        // RAW DATA COLLECTION HEADER
        rawDataFile = new KFileWriter("ColorRawData", opModeUtilities);
        rawDataFile.writeLine("Timestamp,Sensor,Color,Hue,Saturation,Value");

        waitForStart();
        while(opModeIsActive()) {
            detectColorsAction.update();
            rawValues = detectColorsAction.getRawValues();

            if (kPad.isButtonAFirstPressed()) {
                currentColor = NONE;
                KLog.d("kPad", "Color Set to NONE");
            }
            if (kPad.isButtonBFirstPressed()) {
                currentColor = GREEN;
                KLog.d("kPad", "Color Set to GREEN");
            }
            if (kPad.isButtonXFirstPressed()) {
                currentColor = PURPLE;
                KLog.d("kPad", "Color Set to PURPLE");
            }

            // Display current color selection
            if (currentColor != null) {
                telemetry.addLine("Color Set to " + currentColor);
            }

            if (kPad.isToggleY()) {

                // getting calibrated values
                HSV rawValue = rawValues.get(ColorSensorPosition.FRONT);
                calibrate(rawValue, currentColor, calibratedFront);

                rawValue = rawValues.get(ColorSensorPosition.BRIGHT);
                calibrate(rawValue, currentColor, calibratedBRight);

                rawValue = rawValues.get(ColorSensorPosition.BLEFT);
                calibrate(rawValue, currentColor, calibratedBLeft);

                telemetry.addLine("Calibrating...");

                // RAW DATA COLLECTION MAIN
                writeRawDataPoint(detectColorsAction.getRawValues().get(ColorSensorPosition.FRONT), ColorSensorPosition.FRONT, currentColor);
                writeRawDataPoint(detectColorsAction.getRawValues().get(ColorSensorPosition.BLEFT), ColorSensorPosition.BLEFT, currentColor);
                writeRawDataPoint(detectColorsAction.getRawValues().get(ColorSensorPosition.BRIGHT), ColorSensorPosition.BRIGHT, currentColor);

            } else {
                telemetry.addLine("Calibration Stopped.");
            }

            if (kPad.isStartButtonFirstPressed()) {
                saveCalibrationData();
                telemetry.addLine("Calibration Saved!");
            }

            if (kPad.isLeftBumperFirstPressed()) {
                // DETECTING COLORS TEST
                HashMap<ColorSensorPosition, MotifColor> colors = detectColorsAction.getCalculatedColorValue();
                telemetry.addData("RevColorTest", "Front Color: " + colors.get(ColorSensorPosition.FRONT));
                telemetry.addData("RevColorTest", "BLeft Color: " +  colors.get(ColorSensorPosition.BLEFT));
                telemetry.addData("RevColorTest", "BRight Color: " + colors.get(ColorSensorPosition.BRIGHT));
                KLog.d("RevColorTest", "Front Color: " + colors.get(ColorSensorPosition.FRONT));
                KLog.d("RevColorTest", "BLeft Color: " +  colors.get(ColorSensorPosition.BLEFT));
                KLog.d("RevColorTest", "BRight Color: " + colors.get(ColorSensorPosition.BRIGHT));
            }
            telemetry.update();
        }
        rawDataFile.close();
    }

    private void calibrate(HSV rawValue,
                           MotifColor currentColor,
                           HashMap<MotifColor, HSV> calibrationMap) {

            HSV existingEntry = calibrationMap.get(currentColor);
            if (existingEntry == null) {
                existingEntry = new HSV(rawValue.getHue(), rawValue.getSaturation(), rawValue.getValue());
                calibrationMap.put(currentColor, existingEntry);
            } else {
                existingEntry.avgHSV(rawValue.getHue(), rawValue.getSaturation(), rawValue.getValue());
            }
    }

    private void saveCalibrationData() {
        File path = new File(opModeUtilities.getHardwareMap().appContext.getExternalFilesDir(null), "OdometryLog");
        if (!path.exists()) {
            path.mkdirs();
        }

        File file = new File(path, CALIBRATION_FILENAME);

        try {
            FileWriter writer = new FileWriter(file, false);
            writer.write("Sensor,Color,Hue,Saturation,Value\n");

            // Write Front sensor data
            writeCalibrationEntry(writer, ColorSensorPosition.FRONT, PURPLE, calibratedFront.get(PURPLE));
            writeCalibrationEntry(writer, ColorSensorPosition.FRONT, GREEN, calibratedFront.get(GREEN));
            writeCalibrationEntry(writer, ColorSensorPosition.FRONT, NONE, calibratedFront.get(NONE));

            // Write BLeft sensor data
            writeCalibrationEntry(writer, ColorSensorPosition.BLEFT, PURPLE, calibratedBLeft.get(PURPLE));
            writeCalibrationEntry(writer, ColorSensorPosition.BLEFT, GREEN, calibratedBLeft.get(GREEN));
            writeCalibrationEntry(writer, ColorSensorPosition.BLEFT, NONE, calibratedBLeft.get(NONE));

            // Write BRight sensor data
            writeCalibrationEntry(writer, ColorSensorPosition.BRIGHT, PURPLE, calibratedBRight.get(PURPLE));
            writeCalibrationEntry(writer, ColorSensorPosition.BRIGHT, GREEN, calibratedBRight.get(GREEN));
            writeCalibrationEntry(writer, ColorSensorPosition.BRIGHT, NONE, calibratedBRight.get(NONE));

            writer.close();
            KLog.d("ColorCalibration", "Calibration data saved successfully");
            DetectColorsAction.clearCalibratedMap();
        } catch (IOException e) {
            KLog.d("ColorCalibration", "Failed to save calibration data: " + e.getMessage());
        }
    }

    private void writeCalibrationEntry(FileWriter writer, ColorSensorPosition sensor, MotifColor color, HSV hsv) throws IOException {
        if (hsv != null) {
            writer.write(sensor.toString() + "," + color + "," + hsv.getHue() + "," + hsv.getSaturation() + "," + hsv.getValue() + "\n");
        }
    }

    private void writeRawDataPoint(HSV hsv, ColorSensorPosition sensor, MotifColor currentColor) {

        long timestamp = System.currentTimeMillis();
        String colorName = (currentColor != null) ? currentColor.toString() : "UNKNOWN";

        // values collected for rawData, remember to change header if any values here change
        rawDataFile.writeLine(timestamp + "," + sensor.toString() + "," + colorName + "," +
                hsv.getHue() + "," + hsv.getSaturation() + "," + hsv.getValue());
    }

}
