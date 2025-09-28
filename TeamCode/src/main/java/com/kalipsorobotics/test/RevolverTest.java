package com.kalipsorobotics.test;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class RevolverTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        Servo servo = opModeUtilities.getHardwareMap().servo.get("");

        double servoPosition = 0.0;

        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(servoPosition);

            if (gamepad1.a) {
                servoPosition += 1.0/3;
            } else if (gamepad1.b) {
                servoPosition -= 1.0/3;
            }

            if (servoPosition > 1) {
                servoPosition = 1;
            } else if (servoPosition < 0) {
                servoPosition = 0;
            }
        }
    }
}
