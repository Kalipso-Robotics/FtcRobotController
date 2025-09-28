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
        RevColorSensorV3 revColor = hardwareMap.get(RevColorSensorV3.class, "revColor");

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addLine("" + KColor.classify(revColor));
            telemetry.update();
        }

    }
}
