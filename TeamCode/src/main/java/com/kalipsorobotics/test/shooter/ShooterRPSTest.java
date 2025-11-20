package com.kalipsorobotics.test.shooter;

import com.kalipsorobotics.modules.shooter.ShooterConfig;
import com.kalipsorobotics.utilities.KFileWriter;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KMotor;
import com.kalipsorobotics.utilities.OpModeUtilities;
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
 * - Tests from 20 RPS to 54 RPS in 2 RPS increments
 * - Measures ramp-up time to reach target
 * - Maintains target for 1 second and observes stability
 * - Logs: target, ramp-up time, min/max/avg RPS during observation
 *
 * Output CSV format:
 * targetRPS,rampUpTimeMs,minRPS,maxRPS,avgRPS
 */
@TeleOp(name = "Shooter RPS Test", group = "Test")
public class ShooterRPSTest extends LinearOpMode {

    // Test parameters
    private static final double RPS_START = 20.0;
    private static final double RPS_END = 54.0;
    private static final double RPS_STEP = 2.0;
    private static final double RPS_TOLERANCE = 0.5;  // ±0.5 RPS to consider "reached"
    private static final long MAX_RAMP_TIME_MS = 8000;  // Max 8 seconds to reach target
    private static final long OBSERVATION_TIME_MS = 1000;  // Observe for 1 second after reaching target
    private static final long SAMPLE_INTERVAL_MS = 50;  // Sample every 50ms

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
        fileWriter.writeLine("targetRPS,rampUpTimeMs,minRPS,maxRPS,avgRPS");

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

            telemetry.addLine("=== Shooter RPS Test ===");
            telemetry.addData("Test", "%d / %d", testNumber, totalTests);
            telemetry.addData("Target RPS", "%.1f", targetRPS);
            telemetry.update();

            KLog.d("ShooterRPSTest", String.format("========== Test %d/%d: %.1f RPS ==========",
                testNumber, totalTests, targetRPS));

            // Run single test
            TestResult result = runSingleTest(targetRPS);

            // Log results to file
            String line = String.format("%.1f,%.0f,%.2f,%.2f,%.2f",
                targetRPS, result.rampUpTimeMs, result.minRPS, result.maxRPS, result.avgRPS);
            fileWriter.writeLine(line);

            // Display results
            telemetry.addLine();
            telemetry.addData("Ramp-up Time", "%.0f ms", result.rampUpTimeMs);
            telemetry.addData("Min RPS", "%.2f", result.minRPS);
            telemetry.addData("Max RPS", "%.2f", result.maxRPS);
            telemetry.addData("Avg RPS", "%.2f", result.avgRPS);
            telemetry.addData("Reached Target", result.reachedTarget ? "YES" : "NO");
            telemetry.update();

            KLog.d("ShooterRPSTest", String.format(
                "Result: Ramp=%.0fms | Min=%.2f | Max=%.2f | Avg=%.2f | Reached=%b",
                result.rampUpTimeMs, result.minRPS, result.maxRPS, result.avgRPS, result.reachedTarget));

            // Brief pause between tests
            sleep(500);
        }

        // Stop motors
        shooter1.stop();
        shooter2.stop();

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
     * 1. Reset motors
     * 2. Command goToRPS and measure time to reach target
     * 3. Once at target, observe for 1 second and collect RPS data
     * 4. Stop motors and wait for them to spin down
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

        while (opModeIsActive() && timer.milliseconds() < MAX_RAMP_TIME_MS) {
            // Continuously call goToRPS to update PID control
            shooter1.goToRPS(targetRPS);
            shooter2.goToRPS(targetRPS);

            double currentRPS = shooter1.getRPS();

            // Update telemetry
            telemetry.addData("Current RPS", "%.2f", currentRPS);
            telemetry.addData("Target RPS", "%.1f", targetRPS);
            telemetry.addData("Time", "%.1f s", timer.milliseconds() / 1000.0);
            telemetry.update();

            // Check if we've reached target (within ±0.5 RPS)
            if (Math.abs(currentRPS - targetRPS) <= RPS_TOLERANCE) {
                reachedTarget = true;
                result.rampUpTimeMs = timer.milliseconds();
                result.reachedTarget = true;
                KLog.d("ShooterRPSTest", String.format(
                    "Reached target %.1f RPS in %.0fms (actual: %.2f)",
                    targetRPS, result.rampUpTimeMs, currentRPS));
                break;
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
        // Collect RPS data for 1 second after reaching target
        // This measures stability: min, max, and average RPS
        ArrayList<Double> rpsData = new ArrayList<>();

        if (reachedTarget) {
            long startTime = System.currentTimeMillis();
            long endTime = startTime + OBSERVATION_TIME_MS;

            telemetry.addLine();
            telemetry.addLine("Observing stability...");
            telemetry.update();

            while (System.currentTimeMillis() < endTime && opModeIsActive()) {
                // Continue calling goToRPS to maintain target
                shooter1.goToRPS(targetRPS);
                shooter2.goToRPS(targetRPS);

                double currentRPS = shooter1.getRPS();
                rpsData.add(currentRPS);

                // Update telemetry
                telemetry.addData("Current RPS", "%.2f", currentRPS);
                telemetry.addData("Target RPS", "%.1f", targetRPS);
                telemetry.addData("Samples", rpsData.size());
                telemetry.update();

                sleep(SAMPLE_INTERVAL_MS);
            }

            // Calculate statistics
            if (!rpsData.isEmpty()) {
                result.minRPS = calculateMin(rpsData);
                result.maxRPS = calculateMax(rpsData);
                result.avgRPS = calculateAverage(rpsData);
            } else {
                result.minRPS = 0.0;
                result.maxRPS = 0.0;
                result.avgRPS = 0.0;
            }
        } else {
            // Target not reached - use final RPS
            double finalRPS = shooter1.getRPS();
            result.minRPS = finalRPS;
            result.maxRPS = finalRPS;
            result.avgRPS = finalRPS;
        }

        // ========== MOTOR STOP & COOLDOWN ==========
        // Stop motors with brake mode for faster deceleration
        shooter1.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter1.stop();
        shooter2.stop();

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
            ShooterConfig.kp, ShooterConfig.ki, ShooterConfig.kd, ShooterConfig.kf, ShooterConfig.kfBase);

        // Initialize shooter2
        DcMotor motor2 = hardwareMap.dcMotor.get("shooter2");
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter2 = new KMotor(motor2,
            ShooterConfig.kp, ShooterConfig.ki, ShooterConfig.kd, ShooterConfig.kf, ShooterConfig.kfBase);

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
