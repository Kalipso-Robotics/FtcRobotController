package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//add any test stuff you need to do here
@TeleOp
public class TestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor intake = hardwareMap.dcMotor.get("intake");
        Servo holderClamp = hardwareMap.servo.get("holderClamp");
        Servo arm = hardwareMap.servo.get("arm");

        DcMotor lFront = hardwareMap.dcMotor.get("fLeft");
        DcMotor rFront = hardwareMap.dcMotor.get("fRight");
        DcMotor lBack = hardwareMap.dcMotor.get("bLeft");
        DcMotor rBack = hardwareMap.dcMotor.get("bRight");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        rBack.setDirection(DcMotorSimple.Direction.FORWARD);
        lBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rFront.setDirection(DcMotorSimple.Direction.FORWARD);
        lFront.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        double holderClampPos = 0.5;
        //double armPos = 0.5;

        while (opModeIsActive()) {
            if (gamepad1.a == true) {
                intake.setPower(0.6);
            } else if (gamepad1.b == true) {
                intake.setPower(-0.6);
            } else {
                intake.setPower(0);
            }

            if(gamepad1.left_bumper == true) {
                holderClampPos += 0.0025;
            } else if (gamepad1.right_bumper == true) {
                holderClampPos -= 0.0025;
            }

            if(gamepad1.x == true) {
                //armPos += 0.025;
            } else if (gamepad1.y == true) {
                //armPos -= 0.025;
            }

            double forwardBackward = gamepad1.left_stick_y * -0.5;
            double turning = gamepad1.right_stick_x * 0.5;
            double mecanuming = gamepad1.left_stick_x * 0.5;

            double fLeftPower = forwardBackward + turning + mecanuming;
            double fRightPower = forwardBackward - turning - mecanuming;
            double bLeftPower = forwardBackward + turning - mecanuming;
            double bRightPower = forwardBackward - turning + mecanuming;

            double maxPower = maxAbsValueDouble(fLeftPower, bLeftPower, fRightPower, bRightPower);

            if (Math.abs(maxPower) > 1) {
                double scale = Math.abs(maxPower);

                fLeftPower /= scale;
                bLeftPower /= scale;
                fRightPower /= scale;
                bRightPower /= scale;
            }

            lFront.setPower(fLeftPower);
            rFront.setPower(fRightPower);
            lBack.setPower(bLeftPower);
            rBack.setPower(bRightPower);

            holderClamp.setPosition(holderClampPos);
            //arm.setPosition(armPos);
        }
    }

    private double maxAbsValueDouble(double a, double... others) {
        double max = a;

        for (double next : others) {
            if (Math.abs(next) > Math.abs(max)) {
                max = next;
            }
        }

        return max;
    }
}