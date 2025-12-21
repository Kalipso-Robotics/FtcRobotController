package com.kalipsorobotics.test;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class HoodServoToZero extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        Servo hood = opModeUtilities.getHardwareMap().servo.get("hood");
        hood.setPosition(0);

        waitForStart();
        while(opModeIsActive()) {
            hood.setPosition(0);
        }
    }
}
