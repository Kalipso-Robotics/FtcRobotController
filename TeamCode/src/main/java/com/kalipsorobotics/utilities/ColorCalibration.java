package com.kalipsorobotics.utilities;

import static com.kalipsorobotics.utilities.KColor.Color.PURPLE;
import static com.kalipsorobotics.utilities.KColor.Color.GREEN;
import static com.kalipsorobotics.utilities.KColor.Color.NONE;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

@TeleOp
public class ColorCalibration extends LinearOpMode {
    KColor.Color currentColor;
    KGamePad kPad;
    RevColorSensorV3 front;
    RevColorSensorV3 bLeft;
    RevColorSensorV3 bRight;
    HashMap<KColor.Color, HSV> rev1 = new HashMap<>();
    HashMap<KColor.Color, HSV> rev2 = new HashMap<>();
    HashMap<KColor.Color, HSV> rev3 = new HashMap<>();

    @Override
    public void runOpMode() throws InterruptedException {

        front = hardwareMap.get(RevColorSensorV3.class, "revColor1");
        bLeft = hardwareMap.get(RevColorSensorV3.class, "revColor2");
        bRight = hardwareMap.get(RevColorSensorV3.class, "revColor3");

        kPad = new KGamePad(gamepad1);

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
            if (kPad.isToggleButtonY()) {
                calibrate(front, currentColor, rev1);
                calibrate(bLeft, currentColor, rev2);
                calibrate(bRight, currentColor, rev3);
                telemetry.addLine("Calibrating...");
            }

            if (kPad.isToggleRightBumper()) {
                telemetry.addLine("Rev 1 Values, PURPLE: " + rev1.get(PURPLE) + "GREEN: " + rev1.get(GREEN) + "NONE: " + rev1.get(NONE));
                telemetry.addLine("Rev 2 Values, PURPLE: " + rev2.get(PURPLE) + "GREEN: " + rev2.get(GREEN) + "NONE: " + rev2.get(NONE));
                telemetry.addLine("Rev 3 Values, PURPLE: " + rev3.get(PURPLE) + "GREEN: " + rev3.get(GREEN) + "NONE: " + rev3.get(NONE));
            }

            telemetry.update();
        }
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
