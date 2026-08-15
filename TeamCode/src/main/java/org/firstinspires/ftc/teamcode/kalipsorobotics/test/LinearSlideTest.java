package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

@Config
@TeleOp
public class LinearSlideTest extends LinearOpMode {
    private DcMotor linearSlideMotor;
    private OpModeUtilities opModeUtilities;

    // Static config variables - live-editable via FTC Dashboard at runtime
    public static double P = 0.015;
    public static double I = 0.0;
    public static double D = 0.0001;
    public static double F = 0.04;

    private double integralSum = 0;
    private double lastTime = 0;
    private double lastError = 0;

    public double calculate(double target, double current) {
        double currentTime = System.nanoTime() / 1E9;
        double deltaTime = currentTime - lastTime;

        if (lastTime == 0) deltaTime = 0;

        double error = target - current;
        double proportionalOut = P * error;

        integralSum += error * deltaTime;
        if (integralSum > 100) integralSum = 100;
        if (integralSum < -100) integralSum = -100;

        double integralOut = I * integralSum;
        double derivative = deltaTime > 0 ? (error - lastError) / deltaTime : 0;
        double derivativeOut = D * derivative;

        // Feedforward holding power: only active if we are actually trying to stay off the ground
        double feedForwardOut = (target > 10) ? F : 0;

        double power = proportionalOut + integralOut + derivativeOut + feedForwardOut;

        lastError = error;
        lastTime = currentTime;
        return power;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        linearSlideMotor = opModeUtilities.getHardwareMap().dcMotor.get("slideMotor");

        // DIRECTION CONTROL FLAG:
        // Push your joystick UP. If the slide physically moves DOWN instead, change FORWARD to REVERSE here.
        linearSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset encoder so current position starts at 0
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // This mode means WE handle the motor power calculation using our PIDF loop,
        // but the encoder cable still feeds live position data into currentPosition!
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double targetPosition = 0;

        waitForStart();
        lastTime = System.nanoTime() / 1E9;

        while (opModeIsActive()) {
            double joystickInput = -gamepad1.left_stick_y;
            double currentPosition = linearSlideMotor.getCurrentPosition();

            if (Math.abs(joystickInput) > 0.05) {
                // Proportional targeting: target scales directly to how hard you push the stick
                targetPosition = currentPosition + (joystickInput * 40);
            } else {
                // Instantly lock target to hold position right where it is when you let go
                targetPosition = currentPosition;
            }

            // Absolute Physical Constraints (0 to 4684 ticks)
            if (targetPosition < 0) {
                targetPosition = 0;
            } else if (targetPosition > 4660) {
                targetPosition = 4660;
            }

            // Calculate PIDF power based on the proportional target
            double motorPower = calculate(targetPosition, currentPosition);

            // Safety cap: Stop it from driving down if it hits the floor hard stop
            if (motorPower < 0 && currentPosition <= 5) {
                motorPower = 0.0;
            }

            linearSlideMotor.setPower(motorPower);

            // Telemetry feedback
            telemetry.addData("Joystick Input", joystickInput);
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Motor Power Sent", motorPower);
            telemetry.update();
        }
    }
}