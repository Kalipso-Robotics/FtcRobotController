package com.kalipsorobotics.test.intake;

import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoController;

@TeleOp
public class ClawIntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        IntakeClaw intakeClaw = new IntakeClaw(opModeUtilities);
        Outtake outtake = new Outtake(opModeUtilities);
        ServoController servoController = intakeClaw.getIntakeBigPivotServo().getServo().getController();

        double intakeLinkagePos = 0.5;
        double intakeBigSweepPos = 0.1;
        double intakeBigPivotPos = 0.1;
        double intakeSmallPivotPos = 0.5;
        double intakeSmallSweepPos = 0.5;
        double intakeClawPos = 0.5;

        intakeClaw.getIntakeLinkageServo().setPosition(0.95);
        intakeClaw.getIntakeBigSweepServo().setPosition(intakeBigSweepPos);
        intakeClaw.getIntakeBigPivotServo().setPosition(intakeBigPivotPos);
        intakeClaw.getIntakeSmallPivotServo().setPosition(intakeSmallPivotPos);
        intakeClaw.getIntakeSmallSweepServo().setPosition(intakeSmallSweepPos );
        intakeClaw.getIntakeClawServo().setPosition(intakeClawPos);

        double outtakeClawPos = 0.5;

        outtake.getOuttakeClaw().setPosition(outtakeClawPos);

        // .461 .178
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.x) {
                //close
                intakeClawPos = intakeClawPos + 0.001;
                intakeClaw.getIntakeClawServo().setPosition(0.365);
            } else if (gamepad1.b) {
                //open
                intakeClawPos = intakeClawPos - 0.001;
                intakeClaw.getIntakeClawServo().setPosition(0.05);
            }


            if (-gamepad1.right_stick_y != 0) {
                intakeLinkagePos += 0.001 * gamepad1.right_stick_y;
                if (intakeLinkagePos < 0.57) {
                    intakeLinkagePos = 0.57;
                }
                if (intakeLinkagePos > 0.95) {
                    intakeLinkagePos = 0.95;
                }
                intakeClaw.getIntakeLinkageServo().setPosition(intakeLinkagePos);
            }

            if (-gamepad1.right_stick_y > 0.5) {
                //extend
                intakeLinkagePos += 0.001;
                intakeClaw.getIntakeLinkageServo().setPosition(0.57);
            } else if (-gamepad1.right_stick_y < -0.5) {
                //retract
                intakeLinkagePos -= 0.001;
                intakeClaw.getIntakeLinkageServo().setPosition(0.95);

            }

            if (-gamepad1.right_stick_x > 0.5) {
                intakeBigSweepPos -= 0.001;
                intakeClaw.getIntakeBigSweepServo().setPosition(intakeBigSweepPos);
            } else if (-gamepad1.right_stick_x < -0.5) {
                intakeBigSweepPos += 0.001;
                intakeClaw.getIntakeBigSweepServo().setPosition(intakeBigSweepPos);

            }

            if (gamepad1.dpad_up) {
                intakeBigPivotPos += 0.001;
                intakeClaw.getIntakeBigPivotServo().setPosition(intakeBigPivotPos);
            } else if (gamepad1.dpad_down) {
                intakeBigPivotPos -= 0.001;
                intakeClaw.getIntakeBigPivotServo().setPosition(intakeBigPivotPos);
            }

            if (gamepad1.y) {
                intakeSmallPivotPos += 0.001;
                intakeClaw.getIntakeSmallPivotServo().setPosition(intakeSmallPivotPos);
            } else if (gamepad1.a) {
                intakeSmallPivotPos -= 0.001;
                intakeClaw.getIntakeSmallPivotServo().setPosition(intakeSmallPivotPos);
            }

            if (gamepad1.right_bumper) {
                intakeSmallSweepPos += 0.001;
                intakeClaw.getIntakeSmallSweepServo().setPosition(intakeSmallSweepPos);
            } else if (gamepad1.left_bumper) {
                intakeSmallSweepPos -= 0.001;
                intakeClaw.getIntakeSmallSweepServo().setPosition(intakeSmallSweepPos);
            }

            if (gamepad1.dpad_left) {
                outtakeClawPos += 0.001;
                outtake.getOuttakeClaw().setPosition(outtakeClawPos);
            } else if (gamepad1.dpad_right) {
                outtakeClawPos -= 0.001;
                outtake.getOuttakeClaw().setPosition(outtakeClawPos);
            }

            if (gamepad1.left_stick_y > 0.3) {

            }


            telemetry.addData("outtakeClawPos", outtakeClawPos);
            telemetry.addData("intakeClawPos", intakeClawPos);
            telemetry.addData("intakeLinkagePos", intakeLinkagePos);
            telemetry.update();
        }

        servoController.pwmDisable();

    }

}