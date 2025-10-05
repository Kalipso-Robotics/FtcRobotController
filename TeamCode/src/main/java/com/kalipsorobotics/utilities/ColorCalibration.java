package com.kalipsorobotics.utilities;

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
    HashMap<KColor.Color, HSV> rev1 = new HashMap<>();
    HashMap<KColor.Color, HSV> rev2 = new HashMap<>();
    HashMap<KColor.Color, HSV> rev3 = new HashMap<>();

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
            if (gamepad1.a) {
                currentColor = KColor.Color.NONE;
                telemetry.addLine("Color Set to NONE");
            }
            if (gamepad1.b) {
                currentColor = KColor.Color.GREEN;
                telemetry.addLine("Color Set to GREEN");

            }
            if (gamepad1.x) {
                currentColor = PURPLE;
                telemetry.addLine("Color Set to PURPLE");

            }
            if (kPad.isButtonYFirstPressed()) {
                calibrate(front, currentColor, rev1);
                calibrate(bLeft, currentColor, rev2);
                calibrate(bRight, currentColor, rev3);
                telemetry.addLine("Calibrating.");
//                telemetry.clear();
//                telemetry.addLine("Calibrating..");
//                telemetry.clear();
//                telemetry.addLine("Calibrating...");
//                kFile.writeLine(
//                        rev1.get(PURPLE) + "," + rev1.get(GREEN) + "," + rev1.get(NONE) + "," +
//                                rev2.get(PURPLE) + "," + rev2.get(GREEN) + "," + rev2.get(NONE) + "," +
//                                rev3.get(PURPLE) + "," + rev3.get(GREEN) + "," + rev3.get(NONE)
//                );
            } else {
                telemetry.addLine("Calibration Stopped.");
            }

            if (kPad.isRightBumperFirstPressed()) {
                telemetry.clear();
                telemetry.addLine("Rev 1 Values, PURPLE: " + rev1.get(PURPLE) + " GREEN: " + rev1.get(GREEN) + " NONE: " + rev1.get(NONE) + "\n");
                telemetry.addLine("Rev 2 Values, PURPLE: " + rev2.get(PURPLE) + " GREEN: " + rev2.get(GREEN) + " NONE: " + rev2.get(NONE)  + "\n");
                telemetry.addLine("Rev 3 Values, PURPLE: " + rev3.get(PURPLE) + " GREEN: " + rev3.get(GREEN) + " NONE: " + rev3.get(NONE)  + "\n");
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
