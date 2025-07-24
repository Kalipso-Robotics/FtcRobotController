package com.kalipsorobotics.summerCamp;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class CarTeleop extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DcMotor driveMotor = opModeUtilities.getHardwareMap().dcMotor.get("driveMotor");
        Servo rightEye = opModeUtilities.getHardwareMap().servo.get("rightEye");
        Servo leftEye = opModeUtilities.getHardwareMap().servo.get("leftEye");

        double rightEyePosition = 0.5;
        double leftEyePosition = 0.5;

        rightEye.setPosition(rightEyePosition);
        leftEye.setPosition(leftEyePosition);

        waitForStart();
        while (opModeIsActive()) {

            if (Math.abs(-gamepad1.left_stick_y) > 0) {
                driveMotor.setPower(-gamepad1.left_stick_y);
            } else {
                driveMotor.setPower(0);
            }

            if (gamepad1.right_bumper) {
                rightEyePosition += 0.02;
                rightEye.setPosition(rightEyePosition);
            }
            if (gamepad1.right_trigger > 0) {
                rightEyePosition -= 0.02;
                rightEye.setPosition(rightEyePosition);
            }

            if (gamepad1.left_bumper) {
                leftEyePosition += 0.02;
                leftEye.setPosition(leftEyePosition);
            }
            if (gamepad1.left_trigger > 0) {
                leftEyePosition -= 0.02;
                leftEye.setPosition(leftEyePosition);
            }
        }
    }
}
