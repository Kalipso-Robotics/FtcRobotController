package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KMotor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/**
 * Shooter RPS Test TeleOp
 *
 * Tests shooter motors systematically across RPS range:
 * - Tests from 26 RPS to 50 RPS in 0.5 RPS increments
 * - Measures ramp-up time to FIRST reach target (requires 3 consecutive samples within ±1.0 RPS)
 * - Observes for 500ms AFTER reaching target to validate stability
 * - Target considered "reached" ONLY if it stays within overshoot tolerance during observation
 * - Logs: target, ramp-up time, min/max/avg RPS during observation window only
 *
 * Tolerance Settings:
 * - Base tolerance: ±1.0 RPS (must be larger than quantization step of 0.714 RPS)
 * - Overshoot tolerance: ±2.0 RPS (2x base tolerance, allowed during observation)
 * - If RPS exceeds overshoot tolerance during observation, target marked as NOT reached
 *
 * IMPORTANT: Velocity quantization means motor.getVelocity() returns integer ticks/sec
 * With 28 ticks/rotation, RPS can only be multiples of ~0.714 RPS
 *
 * Output CSV format:
 * targetRPS,rampUpTimeMs,minRPS,maxRPS,avgRPS,kp,ki,kd,kf
 */
@TeleOp(name = "Shooter RPS Test", group = "Test")
@Disabled
public class ShooterRPSTest extends LinearOpMode {

    // Test parameters
    private static final double RPS_START = 26;
    private static final double RPS_END = 50;
    private static final double RPS_STEP = 0.5;
    // Velocity quantization: motor.getVelocity() returns integer ticks/sec
    // With 28 ticks/rotation, quantization step = 20 ticks/sec / 28 = 0.714 RPS
    // Tolerance must be larger than quantization to reliably detect target
    private static final double RPS_TOLERANCE = 1.0;  // ±1.0 RPS to consider "reached"
    private static final double OVERSHOOT_TOLERANCE_MULTIPLIER = 2.0;  // Allow 2x tolerance for overshoot
    private static final int CONSECUTIVE_SAMPLES_REQUIRED = 3;  // Must be in tolerance for 3 consecutive samples
    private static final long MAX_RAMP_TIME_MS = 8000;  // Max 8 seconds to reach target
    private static final long OBSERVATION_TIME_MS = 500;  // Observe for 500ms after reaching target
    private static final long SAMPLE_INTERVAL_MS = 50;  // Sample every 50ms
    private static final int MAX_RETRIES = 3;  // Max retries if target not reached

    // Hardware
    private KMotor shooter1;
    private KMotor shooter2;
    private KFileWriter fileWriter;
    private OpModeUtilities opModeUtilities;
    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("=== Shooter RPS Test ===");
        telemetry.addLine("Initializing...");
        telemetry.update();

        // Initialize hardware
        initializeHardware();
        timer = new ElapsedTime();

        // Initialize file writer
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        fileWriter = new KFileWriter("ShooterRPSTestData", opModeUtilities);

        // Write CSV header
        fileWriter.writeLine("targetRPS,rampUpTimeMs,minRPS,maxRPS,avgRPS,kp,ki,kd,kf");

        // Calculate test info
        int totalTests = (int) ((RPS_END - RPS_START) / RPS_STEP) + 1;
        int estimatedTimeMinutes = (totalTests * 10) / 60;  // ~10 seconds per test

        telemetry.addLine("Initialization complete!");
        telemetry.addLine();
        telemetry.addLine("Test Configuration:");
        telemetry.addData("RPS Range", "%.0f to %.0f (step %.0f)", RPS_START, RPS_END, RPS_STEP);
        telemetry.addData("Total Tests", totalTests);
        telemetry.addData("Est. Time", "%d min", estimatedTimeMinutes);
        telemetry.addLine();
        telemetry.addLine("Current PID Values:");
        telemetry.addData("Kp", "%.6f", ShooterConfig.kp);
        telemetry.addData("Ki", "%.6f", ShooterConfig.ki);
        telemetry.addData("Kd", "%.6f", ShooterConfig.kd);
        telemetry.addData("Kf", "%.6f", ShooterConfig.kf);
        telemetry.addLine();
        telemetry.addLine("Press PLAY to start test");
        telemetry.update();

        KLog.d("ShooterRPSTest", "Starting shooter test");
        KLog.d("ShooterRPSTest", String.format("PID: kp=%.6f, ki=%.6f, kd=%.6f, kf=%.6f",
            ShooterConfig.kp, ShooterConfig.ki, ShooterConfig.kd, ShooterConfig.kf));

        waitForStart();

        // ========== MAIN TEST LOOP ==========
        int testNumber = 0;
        for (double targetRPS = RPS_START; targetRPS <= RPS_END + 0.01 && opModeIsActive(); targetRPS += RPS_STEP) {
            testNumber++;

            // Retry loop: retry test if target not reached
            TestResult result = null;
            int attempt = 0;
            boolean success = false;

            while (!success && attempt < MAX_RETRIES && opModeIsActive()) {
                attempt++;

                telemetry.addLine("=== Shooter RPS Test ===");
                telemetry.addData("Test", "%d / %d", testNumber, totalTests);
                telemetry.addData("Target RPS", "%.1f", targetRPS);
                if (attempt > 1) {
                    telemetry.addData("Retry Attempt", "%d / %d", attempt, MAX_RETRIES);
                }
                telemetry.update();

                KLog.d("ShooterRPSTest", String.format("========== Test %d/%d: %.1f RPS (Attempt %d) ==========",
                    testNumber, totalTests, targetRPS, attempt));

                // Run single test
                result = runSingleTest(targetRPS);

                // Check if target was reached
                if (result.reachedTarget) {
                    success = true;
                    KLog.d("ShooterRPSTest", "Test successful on attempt " + attempt);
                } else {
                    KLog.d("ShooterRPSTest", String.format(
                        "Test failed on attempt %d - target not reached within %dms",
                        attempt, MAX_RAMP_TIME_MS));

                    if (attempt < MAX_RETRIES) {
                        telemetry.addLine();
                        telemetry.addLine("Target not reached - retrying...");
                        telemetry.update();
                        sleep(1000);  // Brief pause before retry
                    }
                }
            }

            // Log final result (from last attempt) to file
            if (result != null) {
                String line = String.format("%.1f,%.0f,%.2f,%.2f,%.2f,%.6f,%.6f,%.6f,%.6f",
                    targetRPS, result.rampUpTimeMs, result.minRPS, result.maxRPS, result.avgRPS,
                    ShooterConfig.kp, ShooterConfig.ki, ShooterConfig.kd, ShooterConfig.kf);
                fileWriter.writeLine(line);

                // Display results
                telemetry.addLine();
                telemetry.addData("Attempts", attempt);
                telemetry.addData("Ramp-up Time", "%.0f ms", result.rampUpTimeMs);
                telemetry.addData("Min RPS", "%.2f", result.minRPS);
                telemetry.addData("Max RPS", "%.2f", result.maxRPS);
                telemetry.addData("Avg RPS", "%.2f", result.avgRPS);
                telemetry.addData("Reached Target", result.reachedTarget ? "YES" : "NO");
                telemetry.update();

                KLog.d("ShooterRPSTest", String.format(
                    "Final Result: Attempts=%d | Ramp=%.0fms | Min=%.2f | Max=%.2f | Avg=%.2f | Reached=%b",
                    attempt, result.rampUpTimeMs, result.minRPS, result.maxRPS, result.avgRPS, result.reachedTarget));
            }

            // Brief pause between tests
            sleep(500);
        }

        // Stop motors
        shooter1.stopAndResetPID();
        shooter2.stopAndResetPID();

        // Close file
        fileWriter.close();

        // Display final results
        telemetry.addLine("=== TEST COMPLETE ===");
        telemetry.addLine();
        telemetry.addData("Tests Completed", testNumber);
        telemetry.addLine();
        telemetry.addLine("File saved to:");
        telemetry.addLine("/sdcard/Android/data/");
        telemetry.addLine("com.qualcomm.ftcrobotcontroller/");
        telemetry.addLine("files/OdometryLog/");
        telemetry.update();

        KLog.d("ShooterRPSTest", String.format("Test complete! %d tests finished", testNumber));

        // Keep telemetry visible
        while (opModeIsActive()) {
            sleep(100);
        }
    }

    /**
     * Run a single test at a specific target RPS
     *
     * Process:
     * 1. Reset motors and PID state
     * 2. Ramp-up phase: Command goToRPS and measure time to FIRST reach target
     *    - Requires 3 consecutive samples within ±1.0 RPS tolerance
     *    - Records rampUpTimeMs when target first reached
     * 3. Observation phase (ONLY if target reached during ramp-up):
     *    - Observe for 500ms AFTER reaching target
     *    - Validate RPS stays within overshoot tolerance (±2.0 RPS)
     *    - If excessive deviation occurs, mark target as NOT reached
     *    - Collect min/max/avg RPS during this observation window
     * 4. Stop motors and wait for them to spin down
     *
     * Target is considered "reached" if:
     * - Reached within timeout during ramp-up (3 consecutive samples within ±1.0 RPS)
     * - AND stayed within overshoot tolerance during observation (±2.0 RPS)
     *
     * If either condition fails: reachedTarget = false
     *
     * @param targetRPS The target RPS to test
     * @return TestResult containing ramp time and observation statistics
     */
    private TestResult runSingleTest(double targetRPS) throws InterruptedException {
        TestResult result = new TestResult();

        // Reset PID state for clean start
        shooter1.resetPID();
        shooter2.resetPID();

        // Brief pause to ensure motors are stopped
        sleep(200);

        // ========== RAMP-UP PHASE ==========
        // Command goToRPS until target is reached (within tolerance)
        timer.reset();
        boolean reachedTarget = false;
        int consecutiveSamplesInTolerance = 0;

        while (opModeIsActive() && timer.milliseconds() < MAX_RAMP_TIME_MS) {
            // Continuously call goToRPS to update PID control
            shooter1.goToRPS(targetRPS);
            shooter2.goToRPS(targetRPS);

            double currentRPS = shooter1.getRPS();
            double error = Math.abs(currentRPS - targetRPS);

            // Update telemetry
            telemetry.addData("Current RPS", "%.2f", currentRPS);
            telemetry.addData("Target RPS", "%.1f", targetRPS);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("In Tolerance", "%d/%d", consecutiveSamplesInTolerance, CONSECUTIVE_SAMPLES_REQUIRED);
            telemetry.addData("Time", "%.1f s", timer.milliseconds() / 1000.0);
            telemetry.update();

            // Check if current sample is within tolerance (fixed tolerance, not adaptive)
            if (error <= RPS_TOLERANCE) {
                consecutiveSamplesInTolerance++;

                // Require multiple consecutive samples to confirm stabilization
                if (consecutiveSamplesInTolerance >= CONSECUTIVE_SAMPLES_REQUIRED) {
                    reachedTarget = true;
                    result.rampUpTimeMs = timer.milliseconds();
                    result.reachedTarget = true;
                    KLog.d("ShooterRPSTest", String.format(
                        "Reached target %.1f RPS in %.0fms (actual: %.2f, error: %.2f)",
                        targetRPS, result.rampUpTimeMs, currentRPS, error));
                    break;
                }
            } else {
                // Reset counter if we go out of tolerance
                consecutiveSamplesInTolerance = 0;
            }

            sleep(SAMPLE_INTERVAL_MS);
        }

        // If didn't reach target within timeout, record the timeout
        if (!reachedTarget) {
            result.rampUpTimeMs = timer.milliseconds();
            result.reachedTarget = false;
            KLog.d("ShooterRPSTest", String.format(
                "Timeout: Did not reach target RPS %.1f within %dms",
                targetRPS, MAX_RAMP_TIME_MS));
        }

        // ========== OBSERVATION PHASE ==========
        // ONLY observe if target was reached during ramp-up
        // Collect RPS data for 500ms AFTER reaching target to measure stability and overshoot
        // Validate that RPS stays within tolerance during observation
        ArrayList<Double> rpsData = new ArrayList<>();
        boolean stableInObservation = true;  // Track if stable during observation

        if (reachedTarget) {
            long startTime = System.currentTimeMillis();
            long endTime = startTime + OBSERVATION_TIME_MS;
            double maxAllowedOvershoot = targetRPS + (RPS_TOLERANCE * OVERSHOOT_TOLERANCE_MULTIPLIER);
            double minAllowedUndershoot = targetRPS - (RPS_TOLERANCE * OVERSHOOT_TOLERANCE_MULTIPLIER);

            telemetry.addLine();
            telemetry.addLine("Observing stability...");
            telemetry.update();

            while (System.currentTimeMillis() < endTime && opModeIsActive()) {
                // Continue calling goToRPS to maintain target
                shooter1.goToRPS(targetRPS);
                shooter2.goToRPS(targetRPS);

                double currentRPS = shooter1.getRPS();
                rpsData.add(currentRPS);

                // Check if RPS is within allowed overshoot tolerance
                if (currentRPS > maxAllowedOvershoot || currentRPS < minAllowedUndershoot) {
                    stableInObservation = false;
                    KLog.d("ShooterRPSTest", String.format(
                        "Excessive deviation during observation: %.2f RPS (allowed: %.2f to %.2f)",
                        currentRPS, minAllowedUndershoot, maxAllowedOvershoot));
                }

                // Update telemetry
                telemetry.addData("Current RPS", "%.2f", currentRPS);
                telemetry.addData("Target RPS", "%.1f", targetRPS);
                telemetry.addData("Allowed Range", "%.2f - %.2f", minAllowedUndershoot, maxAllowedOvershoot);
                telemetry.addData("Stable", stableInObservation ? "YES" : "NO");
                telemetry.addData("Samples", rpsData.size());
                telemetry.update();

                sleep(SAMPLE_INTERVAL_MS);
            }

            // Calculate statistics from observation window
            if (!rpsData.isEmpty()) {
                result.minRPS = calculateMin(rpsData);
                result.maxRPS = calculateMax(rpsData);
                result.avgRPS = calculateAverage(rpsData);
            } else {
                result.minRPS = 0.0;
                result.maxRPS = 0.0;
                result.avgRPS = 0.0;
            }

            // Update reachedTarget flag based on observation stability
            if (!stableInObservation) {
                result.reachedTarget = false;
                KLog.d("ShooterRPSTest", String.format(
                    "Target marked as NOT reached due to excessive deviation during observation (min=%.2f, max=%.2f, avg=%.2f)",
                    result.minRPS, result.maxRPS, result.avgRPS));
            }
        } else {
            // Target not reached during ramp-up - use final RPS
            double finalRPS = shooter1.getRPS();
            result.minRPS = finalRPS;
            result.maxRPS = finalRPS;
            result.avgRPS = finalRPS;
        }

        // ========== MOTOR STOP & COOLDOWN ==========
        // Stop motors with brake mode for faster deceleration
        shooter1.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter1.stopAndResetPID();
        shooter2.stopAndResetPID();

        telemetry.addLine("Waiting for motors to stop...");
        telemetry.update();

        // Wait for motors to stop (RPS < 0.5)
        while (opModeIsActive() && shooter1.getRPS() > 0.5) {
            sleep(50);
        }

        // Return to float mode for next test
        shooter1.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        return result;
    }

    /**
     * Initialize shooter motors with proper configuration
     */
    private void initializeHardware() {
        // Initialize shooter1
        DcMotor motor1 = hardwareMap.dcMotor.get("shooter1");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1 = new KMotor(motor1,
            ShooterConfig.kp, ShooterConfig.ki, ShooterConfig.kd, ShooterConfig.kf);

        // Initialize shooter2
        DcMotor motor2 = hardwareMap.dcMotor.get("shooter2");
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter2 = new KMotor(motor2,
            ShooterConfig.kp, ShooterConfig.ki, ShooterConfig.kd, ShooterConfig.kf);

        KLog.d("ShooterRPSTest", "Motors initialized successfully");
    }

    /**
     * Calculate minimum value from list
     */
    private double calculateMin(ArrayList<Double> data) {
        if (data.isEmpty()) return 0.0;
        double min = data.get(0);
        for (double value : data) {
            if (value < min) min = value;
        }
        return min;
    }

    /**
     * Calculate maximum value from list
     */
    private double calculateMax(ArrayList<Double> data) {
        if (data.isEmpty()) return 0.0;
        double max = data.get(0);
        for (double value : data) {
            if (value > max) max = value;
        }
        return max;
    }

    /**
     * Calculate average value from list
     */
    private double calculateAverage(ArrayList<Double> data) {
        if (data.isEmpty()) return 0.0;
        double sum = 0.0;
        for (double value : data) {
            sum += value;
        }
        return sum / data.size();
    }

    /**
     * Simple class to hold test results for one target RPS
     */
    private static class TestResult {
        double rampUpTimeMs = 0.0;  // Time to reach target (milliseconds)
        double minRPS = 0.0;        // Minimum RPS during observation window
        double maxRPS = 0.0;        // Maximum RPS during observation window
        double avgRPS = 0.0;        // Average RPS during observation window
        boolean reachedTarget = false;  // True if target was reached
    }
}
