package com.kalipsorobotics.test;

import com.kalipsorobotics.utilities.KColor;
import com.kalipsorobotics.utilities.KFileWriter;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class ColorTest extends LinearOpMode {


    KColor.Color currentColor;

    @Override
    public void runOpMode() throws InterruptedException {
        RevColorSensorV3 revColor = hardwareMap.get(RevColorSensorV3.class, "revColor1");
        RevColorSensorV3 revColor2 = hardwareMap.get(RevColorSensorV3.class, "revColor2");
        RevColorSensorV3 revColor3 = hardwareMap.get(RevColorSensorV3.class, "revColor3");

        waitForStart();
        while(opModeIsActive()) {
//            KLog.d("revColor", "rev1" + KColor.classify(revColor) + " " + KColor.printHSV(revColor) + " rev2 " + KColor.classify(revColor2) + " " + KColor.printHSV(revColor2) + " rev3 " + KColor.classify(revColor3) + " " +  KColor.printHSV(revColor3));
            KLog.d("revColor", "rev1" + " " + KColor.printHSV(revColor) + " rev2 " +  " " + KColor.printHSV(revColor2) + " rev3 " + " " +  KColor.printHSV(revColor3));
            telemetry.addLine("revColor1: " + KColor.classify(revColor));
//            telemetry.addLine(KColor.printColor(revColor));
            telemetry.addLine(KColor.printHSV(revColor));
            telemetry.addLine("revColor2: " + KColor.classify(revColor2));
//            telemetry.addLine(KColor.printColor(revColor2));
            telemetry.addLine(KColor.printHSV(revColor2));
            telemetry.addLine("revColor3: " + KColor.classify(revColor3));
//            telemetry.addLine(KColor.printColor(revColor3));
            telemetry.addLine(KColor.printHSV(revColor3));
            telemetry.update();
        }
    }
}
