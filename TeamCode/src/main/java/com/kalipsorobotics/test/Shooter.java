package com.kalipsorobotics.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shooter", group = "Linear OpMode")
public class Shooter extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor shooter1 = hardwareMap.dcMotor.get("shooter1");
        DcMotor shooter2 = hardwareMap.dcMotor.get("shooter2");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        double power = 0;

        Servo pusher = hardwareMap.servo.get("pusher");
        double pusherPosition = 0.5;
        double staticPosition = 0.5;

        waitForStart();

        while (opModeIsActive()) {

//            if (gamepad1.left_stick_y != 0) {
//
//                shooter1.setPower(-gamepad1.left_stick_y);
//                shooter2.setPower(-gamepad1.left_stick_y);
//
//            }


            if (gamepad1.a) {
                pusher.setPosition(pusherPosition);
                pusherPosition += 0.001;
            } else if (gamepad1.b) {
                pusher.setPosition(pusherPosition);
                pusherPosition -= 0.001;
            }

            if (gamepad1.x) {
                staticPosition = pusherPosition;
            }


            if (gamepad1.right_trigger != 0) {
                pusher.setPosition(pusherPosition);
            }

            if (gamepad1.left_trigger != 0) {
                pusher.setPosition(staticPosition);
            }

            if (gamepad1.dpad_up) {
                power += 0.00001;
            } else if (gamepad1.dpad_down) {
                power -= 0.00001;
            }

            if (gamepad1.dpad_left) {
                shooter1.setPower(power);
                shooter2.setPower(power);
            } else {
                shooter1.setPower(0);
                shooter2.setPower(0);
            }
            //0.9219
            //0.64
            telemetry.addData("power", power);
            telemetry.addData("pusher position", pusherPosition);
            telemetry.addData("static position", staticPosition);
            telemetry.update();

        }

    }

}
