package com.kalipsorobotics.test;

import com.kalipsorobotics.utilities.KColor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class ColorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RevColorSensorV3 revColor = hardwareMap.get(RevColorSensorV3.class, "revColor1");
        RevColorSensorV3 revColor2 = hardwareMap.get(RevColorSensorV3.class, "revColor2");
        RevColorSensorV3 revColor3 = hardwareMap.get(RevColorSensorV3.class, "revColor3");


        waitForStart();
        while(opModeIsActive()) {
            telemetry.addLine("revColor1: " + KColor.classify(revColor));
            telemetry.addLine(KColor.getColor(revColor));
            telemetry.addLine(KColor.getHSV(revColor));
            telemetry.addLine("revColor2: " + KColor.classify(revColor2));
            telemetry.addLine(KColor.getColor(revColor2));
            telemetry.addLine(KColor.getHSV(revColor2));
            telemetry.addLine("revColor3: " + KColor.classify(revColor3));
            telemetry.addLine(KColor.getColor(revColor3));
            telemetry.addLine(KColor.getHSV(revColor3));
            telemetry.update();
        }
    }
}
