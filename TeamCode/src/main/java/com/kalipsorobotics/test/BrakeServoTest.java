package com.kalipsorobotics.test;

import com.kalipsorobotics.actions.drivetrain.ActivateBraking;
import com.kalipsorobotics.actions.drivetrain.ReleaseBraking;
import com.kalipsorobotics.modules.DriveBrake;
import com.kalipsorobotics.utilities.KOpMode;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class BrakeServoTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {


        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveBrake driveBrake = new DriveBrake(opModeUtilities);

        ActivateBraking activateBraking = new ActivateBraking(driveBrake);
        ReleaseBraking releaseBraking = new ReleaseBraking(driveBrake);


        double positionLeft = 0.5;
        double positionRight = 0.5;

        waitForStart();
        while (opModeIsActive()) {


            if (gamepad1.dpad_down) {
                activateBraking.updateCheckDone();
            } else if (gamepad1.dpad_up) {
                activateBraking.updateCheckDone();
            }

            if (gamepad1.a) {
                driveBrake.getBrakeLeft().getServo().setPosition(positionLeft);
                positionLeft += 0.0001;
            } else if (gamepad1.b) {
                driveBrake.getBrakeLeft().getServo().setPosition(positionLeft);
                positionLeft -= 0.0001;
            }

            if (gamepad1.x) {
                driveBrake.getBrakeRight().setPosition(positionRight);
                positionRight += 0.0001;
            } else if (gamepad1.y) {
                driveBrake.getBrakeRight().setPosition(positionRight);
                positionRight -= 0.0001;
            }

            telemetry.addData("brakeLeftPos ", positionLeft);
            telemetry.addData("brakeRightPos ", positionRight);
            telemetry.update();
        }
    }

}
