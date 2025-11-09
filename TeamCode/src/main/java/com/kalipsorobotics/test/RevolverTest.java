package com.kalipsorobotics.test;

import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.modules.TripleColorSensor;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class RevolverTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        TripleColorSensor colorSensors = new TripleColorSensor(opModeUtilities);
        Revolver revolver = new Revolver(opModeUtilities);
        Shooter shooter = new Shooter(opModeUtilities);

        double servoPosition = 0;

        boolean aWasPressed = false;
        boolean bWasPressed = false;

        waitForStart();

        while (opModeIsActive()) {
            servoPosition = gamepad2.left_stick_x;
            revolver.getRevolverServo().setPosition(servoPosition);

//            if (gamepad1.a && !aWasPressed) {
//                aWasPressed = true;
//            } else if (gamepad1.b && !bWasPressed) {
//                bWasPressed = true;
//            }
//
//            if (!gamepad1.a && aWasPressed) {
//                servoPosition += 0.5;
//                aWasPressed = false;
//            } else if (gamepad1.b) {
//                servoPosition -= 0.5;
//                bWasPressed = false;
//            }
//
//            if (servoPosition > 1) {
//                servoPosition = 1;
//            } else if (servoPosition < 0) {
//                servoPosition = 0;
//            }

            telemetry.addLine("servo position: " + servoPosition);
            telemetry.update();
        }
    }
}
