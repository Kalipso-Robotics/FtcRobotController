package com.kalipsorobotics.utilities;

import static com.kalipsorobotics.utilities.ColorCalibrationDetection.detectColor;
import static com.kalipsorobotics.utilities.KColor.Color.PURPLE;
import static com.kalipsorobotics.utilities.KColor.Color.GREEN;
import static com.kalipsorobotics.utilities.KColor.Color.NONE;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    @Override
    public void runOpMode() throws InterruptedException {

        front = hardwareMap.get(RevColorSensorV3.class, "revColor1");
        bLeft = hardwareMap.get(RevColorSensorV3.class, "revColor2");
        bRight = hardwareMap.get(RevColorSensorV3.class, "revColor3");

        opModeUtilities = new OpModeUtilities(hardwareMap, linearOpMode, telemetry);

        kPad = new KGamePad(gamepad1);
//        kFile = new KFileWriter("color", opModeUtilities);
//        kFile.writeLine("Rev1_PURPLE,Rev1_GREEN,Rev1_NONE,Rev2_PURPLE,Rev2_GREEN,Rev2_NONE,Rev3_PURPLE,Rev3_GREEN,Rev3_NONE");

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
                calibrate(front, currentColor, revFront);
                calibrate(bLeft, currentColor, revBLeft);
                calibrate(bRight, currentColor, revBRight);
                telemetry.addLine("Calibrating...");

//                kFile.writeLine(
//                        rev1.get(PURPLE) + "," + rev1.get(GREEN) + "," + rev1.get(NONE) + "," +
//                                rev2.get(PURPLE) + "," + rev2.get(GREEN) + "," + rev2.get(NONE) + "," +
//                                rev3.get(PURPLE) + "," + rev3.get(GREEN) + "," + rev3.get(NONE)
//                );
            } else {
                telemetry.addLine("Calibration Stopped.");
            }

            if (kPad.isRightBumperFirstPressed()) {
                KLog.d("CalibratedRevColor", "Front, PURPLE: " + revFront.get(PURPLE) + " GREEN: " + revFront.get(GREEN) + " NONE: " + revFront.get(NONE) + "\n");
                KLog.d("CalibratedRevColor","BLeft, PURPLE: " + revBLeft.get(PURPLE) + " GREEN: " + revBLeft.get(GREEN) + " NONE: " + revBLeft.get(NONE)  + "\n");
                KLog.d("CalibratedRevColor","Bright, PURPLE: " + revBRight.get(PURPLE) + " GREEN: " + revBRight.get(GREEN) + " NONE: " + revBRight.get(NONE)  + "\n");
//                telemetry.addLine("Rev 1 Values, PURPLE: " + rev1.get(PURPLE) + " GREEN: " + rev1.get(GREEN) + " NONE: " + rev1.get(NONE) + "\n");
//                telemetry.addLine("Rev 2 Values, PURPLE: " + rev2.get(PURPLE) + " GREEN: " + rev2.get(GREEN) + " NONE: " + rev2.get(NONE)  + "\n");
//                telemetry.addLine("Rev 3 Values, PURPLE: " + rev3.get(PURPLE) + " GREEN: " + rev3.get(GREEN) + " NONE: " + rev3.get(NONE)  + "\n");
            }

            if (kPad.isLeftBumperFirstPressed()) {
                KLog.d("RevColorTest", "Front Color: " + detectColor(revFront, front));
                KLog.d("RevColorTest", "BLeft Color: " +  detectColor(revBLeft, bLeft));
                KLog.d("RevColorTest", "BRight Color: " + detectColor(revBRight, bRight));
            }

            telemetry.update();
        }
//        kFile.close();
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


}
