package org.firstinspires.ftc.teamcode.TANVIII;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DriverMode extends LinearOpMode {
    Robot robot;
    DcMotor armMotor;
    Servo leftyServo;
    Servo rightyServo;

    //TODO: gamepadDrive is only drivetrain
    public void driveWithGamepad() {
        final double powerMult = 0.4;
        double straightSpeed = gamepad1.left_stick_y*powerMult;
        double centerTurnSpeed = gamepad1.right_stick_x*powerMult;
        double mecanumSpeed = gamepad1.left_stick_x*powerMult;

        //set power
        double flpr = (straightSpeed) - (centerTurnSpeed) - (mecanumSpeed);
        double frpr = - (straightSpeed) - (centerTurnSpeed) - (mecanumSpeed);
        double blpr = (straightSpeed) - (centerTurnSpeed) + (mecanumSpeed);
        double brpr = -(straightSpeed) - (centerTurnSpeed) + (mecanumSpeed);

        //scaling
        double bigPr = bigAbsVal(flpr, frpr, blpr, brpr);

        if (Math.abs(bigPr) > (powerMult)) {
            double scaleFactor = Math.abs(bigPr);
            flpr /= scaleFactor/powerMult;
            frpr /= scaleFactor/powerMult;
            blpr /= scaleFactor/powerMult;
            brpr /= scaleFactor/powerMult;

        }
        robot.setDrivetrainPower(flpr, frpr, blpr, brpr);

        telemetry.addLine(String.valueOf(robot.getEncoderPosition("fl")));
        telemetry.addLine(String.valueOf(robot.getEncoderPosition("fr")));
        telemetry.addLine(String.valueOf(robot.getEncoderPosition("bl")));
        telemetry.addLine(String.valueOf(robot.getEncoderPosition("br")));
        telemetry.update();

    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        leftyServo = hardwareMap.servo.get("1");
        rightyServo = hardwareMap.servo.get("2");

        //wheel measurements
        final double ppr = 537.7;
        final double motorToWheelRatio = 1.4;
        final double wheelDiaMm = 96;
        final double pi = 3.14159;

        final double wheelCircMm = wheelDiaMm*pi;
        final double wheelDiaIn = wheelDiaMm/25.4; //25.4mm = 1 in
        final double wheelCircIn = wheelDiaIn*pi;
        double tickToIn = motorToWheelRatio * wheelCircIn / ppr;

        //wait
        waitForStart();

        while (opModeIsActive()) {

            driveWithGamepad();

            //moving arm
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                robot.setArmPower(0);
            } else if (gamepad1.right_bumper) {
                robot.setArmPower(-1);
            } else if (gamepad1.left_bumper) {
                robot.setArmPower(1);
            } else {
                robot.setArmPower(0);
            }

            //move servos
            if (gamepad1.a) {
                leftyServo.setPosition(0);
                rightyServo.setPosition(1);
            } else if (gamepad1.b) {
                leftyServo.setPosition(1);
                rightyServo.setPosition(0);
            }
        }
    }
    private double bigAbsVal(double a, double... others) {

        double max = a;

        for (double next : others) {
            if (Math.abs(next) > Math.abs(max)) {
                max = next;
            }
        }
        return max;
    }
}