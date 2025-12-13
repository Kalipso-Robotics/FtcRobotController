package com.kalipsorobotics.test.turret;

import com.kalipsorobotics.actions.turret.TurretConfig;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.KFileWriter;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KMotor;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/**
 * Turret PID Data Collector
 *
 * Simple data collection program that tests the turret motor's built-in PID controller
 * using the PID values from TurretConfig across a range of target positions.
 *
 * APPROACH:
 * =========
 * - Uses motor's RUN_TO_POSITION mode with TurretConfig PID values
 * - Tests each target position from -750 to +750 ticks
 * - Measures time to reach target and stability (min/max/avg) for 1 second after
 * - Writes data immediately after each test (survives crashes/battery loss)
 *
 * DATA COLLECTED:
 * ===============
 * - kp, ki, kd: PID values from TurretConfig
 * - targetTicks: Target position
 * - timeToTarget: Time to reach target (milliseconds)
 * - minTicks: Minimum position during 1-second observation
 * - maxTicks: Maximum position during 1-second observation
 * - avgTicks: Average position during 1-second observation
 *
 * OUTPUT:
 * =======
 * CSV file: kp,ki,kd,targetTicks,timeToTarget,minTicks,maxTicks,avgTicks
 *
 * USAGE:
 * ======
 * 1. Position turret at center before starting
 * 2. Run "Turret PID Data Collector" TeleOp
 * 3. Data is saved after each test (crash-safe)
 * 4. Pull CSV from /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/OdometryLog/
 * 5. Analyze data to tune PID values in TurretConfig
 */
//@TeleOp(name = "Turret PID Data Collector", group = "Test")
public class TurretPIDDataCollector extends LinearOpMode {

    // Target range (in ticks)
    private static final double TARGET_MIN_TICKS = -750.9765625;
    private static final double TARGET_MAX_TICKS = 750.9765625;
    private static final double TARGET_STEP_TICKS = 20;  // Test every 50 ticks

    // Test parameters
    private static final double POSITION_TOLERANCE_TICKS = 10.0;  // Within ±10 ticks
    private static final long MAX_MOVEMENT_TIME_MS = 5000;  // 5 seconds max per movement
    private static final long OBSERVATION_TIME_MS = 1000;  // Observe for 1 second after reaching target
    private static final long SAMPLE_INTERVAL_MS = 20;  // Sample every 20ms

    // Hardware
    private Turret turret;
    private KMotor turretMotor;
    private KFileWriter dataWriter;
    private OpModeUtilities opModeUtilities;
    private ElapsedTime testTimer;

    // PID values from TurretConfig
    private double kp;
    private double ki;
    private double kd;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("=== Turret PID Data Collector ===");
        telemetry.addLine("Initializing...");
        telemetry.update();

        // Initialize hardware
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        turret = Turret.getInstance(opModeUtilities);
        turretMotor = turret.getTurretMotor();
        testTimer = new ElapsedTime();

        // Get PID values from TurretConfig
        kp = TurretConfig.kP;
        ki = TurretConfig.kI;
        kd = TurretConfig.kD;

        // Initialize file writer
        dataWriter = new KFileWriter("TurretPIDDataCollector", opModeUtilities);

        // Write CSV header with units
        dataWriter.writeLine("kp,ki,kd,targetTicks,timeToTarget_ms,minTicks,maxTicks,avgTicks");

        // Calculate total tests
        int totalTests = (int) ((TARGET_MAX_TICKS - TARGET_MIN_TICKS) / TARGET_STEP_TICKS) + 1;

        // Display configuration
        telemetry.addLine("Initialization complete!");
        telemetry.addLine();
        telemetry.addLine("PID values from TurretConfig:");
        telemetry.addData("  kP", "%.6f", kp);
        telemetry.addData("  kI", "%.6f", ki);
        telemetry.addData("  kD", "%.6f", kd);
        telemetry.addLine();
        telemetry.addData("Target range", "%.1f to %.1f ticks", TARGET_MIN_TICKS, TARGET_MAX_TICKS);
        telemetry.addData("Step size", "%.1f ticks", TARGET_STEP_TICKS);
        telemetry.addData("Total tests", "%d", totalTests);
        telemetry.addLine();
        telemetry.addLine("IMPORTANT:");
        telemetry.addLine("- Position turret at CENTER before starting");
        telemetry.addLine("- Data saved after each test (crash-safe)");
        telemetry.addLine();
        telemetry.addLine("Press PLAY to start");
        telemetry.update();

        KLog.d("TurretPIDDataCollector", String.format(
            "Starting data collection: kp=%.6f, ki=%.6f, kd=%.6f, %d tests",
            kp, ki, kd, totalTests));

        waitForStart();

        // Reset encoder to zero (current position = 0)
        // KMotor uses manual PID control, so we need RUN_USING_ENCODER mode
        turretMotor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(100);

        KLog.d("TurretPIDDataCollector", String.format(
            "Motor reset. Using KMotor with TurretConfig PID: kp=%.6f, ki=%.6f, kd=%.6f",
            kp, ki, kd));
        KLog.d("TurretPIDDataCollector", "Starting tests...");

        // Main data collection loop
        int testCounter = 0;

        for (double targetTicks = TARGET_MIN_TICKS;
             targetTicks <= TARGET_MAX_TICKS + 0.01 && opModeIsActive();
             targetTicks += TARGET_STEP_TICKS) {

            testCounter++;

            // Display progress
            telemetry.clear();
            telemetry.addLine("=== Turret PID Data Collector ===");
            telemetry.addData("Test", "%d / %d", testCounter, totalTests);
            telemetry.addData("Current position", "%d ticks", turretMotor.getCurrentPosition());
            telemetry.addData("Target", "%.1f ticks", targetTicks);
            telemetry.update();

            KLog.d("TurretPIDDataCollector", String.format(
                "========== Test %d/%d: target=%.1f ticks ==========",
                testCounter, totalTests, targetTicks));

            // Run test
            TestResult result = runTest(targetTicks);

            // Write data to file immediately (crash-safe)
            String line = String.format("%.6f,%.6f,%.6f,%.1f,%.1f,%.1f,%.1f,%.1f",
                kp, ki, kd,
                targetTicks,
                result.timeToTarget,
                result.minTicks,
                result.maxTicks,
                result.avgTicks);
            dataWriter.writeLine(line);

            // Log results
            KLog.d("TurretPIDDataCollector", String.format(
                "Target: %.1f | Time: %.0fms | Range: [%.1f, %.1f] | Avg: %.1f",
                targetTicks, result.timeToTarget, result.minTicks, result.maxTicks, result.avgTicks));

            telemetry.addData("Time to target", "%.0fms", result.timeToTarget);
            telemetry.addData("Position range", "[%.1f, %.1f]", result.minTicks, result.maxTicks);
            telemetry.update();

            sleep(200);  // Brief pause between tests
        }

        // Stop motor and close file
        turretMotor.setPower(0);
        dataWriter.close();

        // Display completion
        telemetry.clear();
        telemetry.addLine("=== DATA COLLECTION COMPLETE ===");
        telemetry.addLine();
        telemetry.addData("Tests completed", "%d / %d", testCounter, totalTests);
        telemetry.addLine();
        telemetry.addLine("Data saved to:");
        telemetry.addLine("/sdcard/Android/data/");
        telemetry.addLine("com.qualcomm.ftcrobotcontroller/");
        telemetry.addLine("files/OdometryLog/");
        telemetry.update();

        KLog.d("TurretPIDDataCollector", String.format(
            "Data collection complete! Collected %d data points.", testCounter));

        // Keep display visible
        while (opModeIsActive()) {
            sleep(100);
        }
    }

    /**
     * Run a single test: move to target and observe
     */
    private TestResult runTest(double targetTicks) throws InterruptedException {

        TestResult result = new TestResult();

        int targetPosition = (int) Math.round(targetTicks);

        // Wait until target is reached
        testTimer.reset();
        boolean reachedTarget = false;

        // IMPORTANT: Call goToTargetTicks continuously in loop for PID to work
        while (opModeIsActive() && testTimer.milliseconds() < MAX_MOVEMENT_TIME_MS) {
            // Must call this every loop - PID needs continuous updates
            turretMotor.goToTargetTicks(targetPosition);

            int currentPosition = turretMotor.getCurrentPosition();

            // Check if reached target (within tolerance)
            if (Math.abs(currentPosition - targetPosition) <= POSITION_TOLERANCE_TICKS) {
                reachedTarget = true;
                result.timeToTarget = testTimer.milliseconds();

                KLog.d("TurretPIDDataCollector", String.format(
                    "Reached target %d in %.0fms (actual: %d)",
                    targetPosition, result.timeToTarget, currentPosition));
                break;
            }

            sleep(SAMPLE_INTERVAL_MS);
        }

        // If didn't reach target, record timeout
        if (!reachedTarget) {
            result.timeToTarget = testTimer.milliseconds();
            KLog.d("TurretPIDDataCollector", String.format(
                "Timeout: Did not reach target %d within %dms (final: %d)",
                targetPosition, MAX_MOVEMENT_TIME_MS, turretMotor.getCurrentPosition()));
        }

        // Observe for 1 second after reaching target
        ArrayList<Double> observationSamples = new ArrayList<>();
        long observationStart = System.currentTimeMillis();
        long observationEnd = observationStart + OBSERVATION_TIME_MS;

        while (opModeIsActive() && System.currentTimeMillis() < observationEnd) {
            // Continue calling PID to hold position during observation
            turretMotor.goToTargetTicks(targetPosition);

            int currentPosition = turretMotor.getCurrentPosition();
            observationSamples.add((double) currentPosition);
            sleep(SAMPLE_INTERVAL_MS);
        }

        // Calculate statistics from observation period
        if (!observationSamples.isEmpty()) {
            result.minTicks = calculateMin(observationSamples);
            result.maxTicks = calculateMax(observationSamples);
            result.avgTicks = calculateAverage(observationSamples);
        } else {
            int finalPos = turretMotor.getCurrentPosition();
            result.minTicks = finalPos;
            result.maxTicks = finalPos;
            result.avgTicks = finalPos;
        }

        return result;
    }

    // Statistics helpers
    private double calculateMin(ArrayList<Double> data) {
        if (data.isEmpty()) return 0.0;
        double min = data.get(0);
        for (double value : data) {
            if (value < min) min = value;
        }
        return min;
    }

    private double calculateMax(ArrayList<Double> data) {
        if (data.isEmpty()) return 0.0;
        double max = data.get(0);
        for (double value : data) {
            if (value > max) max = value;
        }
        return max;
    }

    private double calculateAverage(ArrayList<Double> data) {
        if (data.isEmpty()) return 0.0;
        double sum = 0.0;
        for (double value : data) {
            sum += value;
        }
        return sum / data.size();
    }

    /**
     * Class to hold test results
     */
    private static class TestResult {
        double timeToTarget = 0.0;
        double minTicks = 0.0;
        double maxTicks = 0.0;
        double avgTicks = 0.0;
    }
}
