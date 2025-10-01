package com.kalipsorobotics.test;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class RevolverTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        Servo servo = opModeUtilities.getHardwareMap().servo.get("revolver");

        double servoPosition = 1.0/3;

        boolean aWasPressed = false;
        boolean bWasPressed = false;

        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(servoPosition);

            if (gamepad1.a && !aWasPressed) {
                aWasPressed = true;
            } else if (gamepad1.b && !bWasPressed) {
                bWasPressed = true;
            }

            if (!gamepad1.a && aWasPressed) {
                servoPosition += 0.33;
                aWasPressed = false;
            } else if (gamepad1.b) {
                servoPosition -= 0.33;
                bWasPressed = false;
            }

            if (servoPosition > 1) {
                servoPosition = 1;
            } else if (servoPosition < 0) {
                servoPosition = 0;
            }

            telemetry.addLine("servo position: " + servoPosition);
            telemetry.update();
        }
    }
}
