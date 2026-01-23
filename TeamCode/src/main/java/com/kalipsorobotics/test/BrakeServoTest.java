package com.kalipsorobotics.test;

import com.kalipsorobotics.actions.drivetrain.ActivateBraking;
import com.kalipsorobotics.actions.drivetrain.ReleaseBraking;
import com.kalipsorobotics.modules.DriveBrake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class BrakeServoTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {


        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveBrake driveBrake = new DriveBrake(opModeUtilities);

        ActivateBraking activateBraking;
        ReleaseBraking releaseBraking;


        double position = 0.5;

        waitForStart();
        while (opModeIsActive()) {


            if (gamepad1.dpad_down) {
                activateBraking = new ActivateBraking(driveBrake);
                activateBraking.updateCheckDone();
            } else if (gamepad1.dpad_up) {
                releaseBraking = new ReleaseBraking(driveBrake);
                releaseBraking.updateCheckDone();

            }

            if (gamepad1.a) {
                driveBrake.getBrake().getServo().setPosition(position);
                position += 0.0001;
            } else if (gamepad1.b) {
                driveBrake.getBrake().getServo().setPosition(position);
                position -= 0.0001;
            }


            telemetry.addData("brakePos ", position);
            telemetry.update();
        }
    }

}
