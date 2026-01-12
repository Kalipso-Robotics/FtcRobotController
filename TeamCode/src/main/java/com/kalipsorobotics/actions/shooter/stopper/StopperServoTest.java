package com.kalipsorobotics.actions.shooter.stopper;

import com.kalipsorobotics.actions.drivetrain.ActivateBraking;
import com.kalipsorobotics.actions.drivetrain.ReleaseBraking;
import com.kalipsorobotics.modules.DriveBrake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.utilities.KOpMode;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class StopperServoTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {


        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        Stopper stopper = new Stopper(opModeUtilities);

        double positionOpen = 0.5;
        double positionClosed = 0.5;

        waitForStart();
        while (opModeIsActive()) {


            if (gamepad1.dpad_left) {
                stopper.getStopper().getServo().setPosition(positionClosed);
            } else if (gamepad1.dpad_right) {
                stopper.getStopper().getServo().setPosition(positionOpen);
            }

            if (gamepad1.a) {
                stopper.getStopper().getServo().setPosition(positionOpen);
                positionOpen += 0.001;
            } else if (gamepad1.b) {
                stopper.getStopper().getServo().setPosition(positionOpen);
                positionOpen -= 0.001;
            }

            if (gamepad1.x) {
                stopper.getStopper().getServo().setPosition(positionClosed);
                positionClosed += 0.001;
            } else if (gamepad1.y){
                stopper.getStopper().setPosition(positionClosed);
                positionClosed -= 0.001;
            }



            telemetry.addData("stopper open pos ", positionOpen);
            telemetry.addData("stopper closed pos ", positionClosed);
            telemetry.update();
        }
    }

}
