package com.kalipsorobotics.test.shooter;

import com.kalipsorobotics.math.CalculateTickPer;
import com.kalipsorobotics.utilities.KLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Basic Shooter Motor Test
 *
 * Simple TeleOp to test shooter1 motor with velocity control.
 * Sets target to 50 RPS and displays encoder position, velocity,
 * and RPS in both telemetry and logcat.
 *
 * Controls:
 * - Press PLAY to start motor at 50 RPS
 * - Press STOP to stop the motor
 */
//@TeleOp(name = "Shooter Motor Test", group = "Test")
public class ShooterMotorTest extends LinearOpMode {

    private static final double TARGET_RPS = 50.0;

    private DcMotorEx shooter1Motor;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("=== Shooter Motor Test ===");
        telemetry.addLine("Initializing motor...");
        telemetry.update();

        // Initialize motor
        shooter1Motor = hardwareMap.get(DcMotorEx.class, "shooter2");

        // Configure motor
        shooter1Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Motor initialized!");
        telemetry.addData("Target RPS", "%.2f", TARGET_RPS);
        telemetry.addLine();
        telemetry.addLine("Press PLAY to start motor");
        telemetry.update();

        KLog.d("ShooterMotorTest", "=== Shooter Motor Test Initialized ===");
        KLog.d("ShooterMotorTest", "Target RPS: " + TARGET_RPS);

        waitForStart();

        // Convert RPS to ticks per second
        double targetTPS = CalculateTickPer.rotationToTicks6000RPM(TARGET_RPS);

        // Set target velocity
        shooter1Motor.setVelocity(targetTPS);

        KLog.d("ShooterMotorTest", "Motor started - Target: " + TARGET_RPS + " RPS (" + targetTPS + " TPS)");

        // Main loop
        while (opModeIsActive()) {
            // Get motor data
            int encoderPosition = shooter1Motor.getCurrentPosition();
            double velocityTPS = shooter1Motor.getVelocity();
            double velocityRPS = CalculateTickPer.ticksToRotation6000RPM(velocityTPS);

            // Calculate error
            double errorRPS = TARGET_RPS - velocityRPS;
            double errorPercent = (errorRPS / TARGET_RPS) * 100.0;

            // Display telemetry
            telemetry.addLine("=== SHOOTER MOTOR TEST ===");
            telemetry.addLine();
            telemetry.addData("Target RPS", "%.2f", TARGET_RPS);
            telemetry.addLine();

            telemetry.addLine("--- Motor Data ---");
            telemetry.addData("Encoder Position", "%d ticks", encoderPosition);
            telemetry.addData("Velocity (TPS)", "%.2f", velocityTPS);
            telemetry.addData("Velocity (RPS)", "%.2f", velocityRPS);
            telemetry.addLine();

            telemetry.addLine("--- Performance ---");
            telemetry.addData("Error", "%.2f RPS (%.1f%%)", errorRPS, errorPercent);
            telemetry.addData("At Target", Math.abs(errorRPS) < 1.0 ? "YES ✓" : "NO");

            telemetry.update();

            // Log to logcat every 500ms
            if (System.currentTimeMillis() % 500 < 50) {
                KLog.d("ShooterMotorTest", String.format("Encoder: %d ticks | Velocity: %.2f TPS (%.2f RPS) | Error: %.2f RPS (%.1f%%)",
                    encoderPosition, velocityTPS, velocityRPS, errorRPS, errorPercent));
            }

            sleep(50);
        }

        // Stop motor when OpMode ends
        shooter1Motor.setVelocity(0);
        shooter1Motor.setPower(0);
        KLog.d("ShooterMotorTest", "Motor stopped");
    }
}