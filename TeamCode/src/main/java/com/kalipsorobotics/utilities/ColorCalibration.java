package com.kalipsorobotics.utilities;

import static com.kalipsorobotics.utilities.KColorDetection.detectColor;
import static com.kalipsorobotics.utilities.KColor.Color.PURPLE;
import static com.kalipsorobotics.utilities.KColor.Color.GREEN;
import static com.kalipsorobotics.utilities.KColor.Color.NONE;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;

@TeleOp
public class ColorCalibration extends LinearOpMode {
    KColor.Color currentColor;
    KGamePad kPad;
    KFileWriter kFile;
    RevColorSensorV3 front;
    RevColorSensorV3 bLeft;
    RevColorSensorV3 bRight;
    OpModeUtilities opModeUtilities;
    LinearOpMode linearOpMode;
    HashMap<KColor.Color, HSV> revFront = new HashMap<>();
    HashMap<KColor.Color, HSV> revBLeft = new HashMap<>();
    HashMap<KColor.Color, HSV> revBRight = new HashMap<>();

    private static final String CALIBRATION_FILENAME = "color_calibration.csv";

    // Uncomment to collect raw data points for analysis
    private KFileWriter rawDataFile;

    @Override
    public void runOpMode() throws InterruptedException {

        bLeft = hardwareMap.get(RevColorSensorV3.class, "revColor1");
        bRight = hardwareMap.get(RevColorSensorV3.class, "revColor2");
        front = hardwareMap.get(RevColorSensorV3.class, "revColor3");

        opModeUtilities = new OpModeUtilities(hardwareMap, linearOpMode, telemetry);

        kPad = new KGamePad(gamepad1);

        // Try to load saved calibration data
        loadCalibrationData();

        // RAW DATA COLLECTION HEADER
        rawDataFile = new KFileWriter("ColorRawData", opModeUtilities);
        rawDataFile.writeLine("Timestamp,Sensor,Color,Hue,Saturation,Value,Distance,Red,Green,Blue");

        waitForStart();
        while(opModeIsActive()) {
            if (kPad.isButtonAFirstPressed()) {
                currentColor = KColor.Color.NONE;
                KLog.d("kPad", "Color Set to NONE");
            }
            if (kPad.isButtonBFirstPressed()) {
                currentColor = KColor.Color.GREEN;
                KLog.d("kPad", "Color Set to GREEN");
            }
            if (kPad.isButtonXFirstPressed()) {
                currentColor = KColor.Color.PURPLE;
                KLog.d("kPad", "Color Set to PURPLE");
            }

            // Display current color selection
            if (currentColor != null) {
                telemetry.addLine("Color Set to " + currentColor);
            }

            if (kPad.isToggleY()) {
                // getting calibrated values
                calibrate(front, currentColor, revFront);
                calibrate(bLeft, currentColor, revBLeft);
                calibrate(bRight, currentColor, revBRight);
                telemetry.addLine("Calibrating...");

                // RAW DATA COLLECTION MAIN
                collectRawDataPoint(front, "Front", currentColor);
                collectRawDataPoint(bLeft, "BLeft", currentColor);
                collectRawDataPoint(bRight, "BRight", currentColor);

            } else {
                telemetry.addLine("Calibration Stopped.");
            }

            if (kPad.isRightBumperFirstPressed()) {
                // SET OF CALLIBRATED COLOR HSV VALUES
                KLog.d("CalibratedRevColor", "Front, PURPLE: " + revFront.get(PURPLE) + " GREEN: " + revFront.get(GREEN) + " NONE: " + revFront.get(NONE) + "\n");
                KLog.d("CalibratedRevColor","BLeft, PURPLE: " + revBLeft.get(PURPLE) + " GREEN: " + revBLeft.get(GREEN) + " NONE: " + revBLeft.get(NONE)  + "\n");
                KLog.d("CalibratedRevColor","Bright, PURPLE: " + revBRight.get(PURPLE) + " GREEN: " + revBRight.get(GREEN) + " NONE: " + revBRight.get(NONE)  + "\n");
//                telemetry.addLine("Rev 1 Values, PURPLE: " + rev1.get(PURPLE) + " GREEN: " + rev1.get(GREEN) + " NONE: " + rev1.get(NONE) + "\n");
//                telemetry.addLine("Rev 2 Values, PURPLE: " + rev2.get(PURPLE) + " GREEN: " + rev2.get(GREEN) + " NONE: " + rev2.get(NONE)  + "\n");
//                telemetry.addLine("Rev 3 Values, PURPLE: " + rev3.get(PURPLE) + " GREEN: " + rev3.get(GREEN) + " NONE: " + rev3.get(NONE)  + "\n");
            }

            if (kPad.isLeftBumperFirstPressed()) {
                // DETECTING COLORS TEST
                KLog.d("RevColorTest", "Front Color: " + detectColor(revFront, front));
                KLog.d("RevColorTest", "BLeft Color: " +  detectColor(revBLeft, bLeft));
                KLog.d("RevColorTest", "BRight Color: " + detectColor(revBRight, bRight));
            }

            // Save calibration when Start button is pressed
            if (kPad.isStartButtonPressed()) {
                saveCalibrationData();
                telemetry.addLine("Calibration Saved!");
            }

            telemetry.update();
        }

        // Uncomment to close raw data file
        rawDataFile.close();
    }

    public static void calibrate(RevColorSensorV3 revColor,
                                 KColor.Color currentColor,
                                 HashMap<KColor.Color, HSV> calibrationMap) {
        int red   = revColor.red();
        int green = revColor.green();
        int blue  = revColor.blue();

        float[] curHSV = new float[3];
        android.graphics.Color.RGBToHSV(red, green, blue, curHSV);

        HSV entry = calibrationMap.get(currentColor);
        if (entry == null) {
            entry = new HSV(curHSV[0], curHSV[1], curHSV[2]);
            calibrationMap.put(currentColor, entry);
        } else {
            entry.avgHSV(curHSV[0], curHSV[1], curHSV[2]);
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
            writeCalibrationEntry(writer, "Front", PURPLE, revFront.get(PURPLE));
            writeCalibrationEntry(writer, "Front", GREEN, revFront.get(GREEN));
            writeCalibrationEntry(writer, "Front", NONE, revFront.get(NONE));

            // Write BLeft sensor data
            writeCalibrationEntry(writer, "BLeft", PURPLE, revBLeft.get(PURPLE));
            writeCalibrationEntry(writer, "BLeft", GREEN, revBLeft.get(GREEN));
            writeCalibrationEntry(writer, "BLeft", NONE, revBLeft.get(NONE));

            // Write BRight sensor data
            writeCalibrationEntry(writer, "BRight", PURPLE, revBRight.get(PURPLE));
            writeCalibrationEntry(writer, "BRight", GREEN, revBRight.get(GREEN));
            writeCalibrationEntry(writer, "BRight", NONE, revBRight.get(NONE));

            writer.close();
            KLog.d("ColorCalibration", "Calibration data saved successfully");
        } catch (IOException e) {
            KLog.d("ColorCalibration", "Failed to save calibration data: " + e.getMessage());
        }
    }

    private void writeCalibrationEntry(FileWriter writer, String sensor, KColor.Color color, HSV hsv) throws IOException {
        if (hsv != null) {
            writer.write(sensor + "," + color + "," + hsv.getHue() + "," + hsv.getSaturation() + "," + hsv.getValue() + "\n");
        }
    }

    private void loadCalibrationData() {
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
                KColor.Color color = KColor.Color.valueOf(values[1]);
                float hue = Float.parseFloat(values[2]);
                float sat = Float.parseFloat(values[3]);
                float val = Float.parseFloat(values[4]);

                HSV hsv = new HSV(hue, sat, val);

                if (sensor.equals("Front")) {
                    revFront.put(color, hsv);
                } else if (sensor.equals("BLeft")) {
                    revBLeft.put(color, hsv);
                } else if (sensor.equals("BRight")) {
                    revBRight.put(color, hsv);
                }
            }

            reader.close();
            KLog.d("ColorCalibration", "Calibration data loaded successfully");
        } catch (Exception e) {
            KLog.d("ColorCalibration", "No existing calibration data found or failed to load: " + e.getMessage());
        }
    }

    private void collectRawDataPoint(RevColorSensorV3 revColor, String sensorName, KColor.Color currentColor) {
        int red = revColor.red();
        int green = revColor.green();
        int blue = revColor.blue();

        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV(red, green, blue, hsv);

        long timestamp = System.currentTimeMillis();
        String colorName = (currentColor != null) ? currentColor.toString() : "UNKNOWN";

        // values collected for rawData, remember to change header if any values here change
        rawDataFile.writeLine(timestamp + "," + sensorName + "," + colorName + "," +
                hsv[0] + "," + hsv[1] + "," + hsv[2] + "," + revColor.getDistance(DistanceUnit.MM) + ","  + red + "," + green + "," + blue);
    }

}
