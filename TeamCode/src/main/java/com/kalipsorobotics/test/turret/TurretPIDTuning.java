package com.kalipsorobotics.test.turret;

import com.kalipsorobotics.actions.turret.TurretConfig;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.KFileWriter;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

/**
 * Turret PID Data Collection System
 *
 * This program collects comprehensive performance data for the turret using
 * the current TurretConfig PID values across a wide range of target positions.
 *
 * APPROACH:
 * =========
 * Instead of brute-forcing all PID combinations, this collects detailed data
 * for ONE set of PID values across many target positions. You can then manually
 * analyze the data to identify patterns and determine smarter PID adjustments.
 *
 * FEATURES:
 * =========
 * - Uses current TurretConfig PID values (kP, kI, kD)
 * - Tests wide range of target positions (-180° to +180°)
 * - Collects detailed metrics for each test
 * - Persistent state saving (survives battery drops/crashes)
 * - 20-minute runtime limit
 * - Logs all data using KLog.d and saves to CSV file
 *
 * TUNABLE PARAMETERS:
 * ===================
 * - MAX_RUNTIME_MS: Maximum runtime (20 minutes)
 * - TARGET_STEP_DEGREES: Step size between target positions (default: 15°)
 * - TARGET_MIN_DEGREES: Minimum target angle (default: -180°)
 * - TARGET_MAX_DEGREES: Maximum target angle (default: +180°)
 * - POSITION_TOLERANCE: Acceptable error range (in ticks)
 * - MAX_MOVEMENT_TIME_MS: Maximum time to reach target
 * - SAMPLES_PER_TEST: How many data points to collect per position test
 *
 * DATA COLLECTED PER TEST:
 * ========================
 * - targetTicks: Target position in encoder ticks
 * - finalTicks: Final position reached
 * - maxTicks: Maximum position during movement
 * - minTicks: Minimum position during movement
 * - avgTicks: Average position across all samples
 * - timeToTarget: Time to reach target (ms)
 * - overshoot: Amount exceeded target (ticks)
 * - steadyStateError: Final error from target (ticks)
 * - oscillationRange: Max - Min during stability window (ticks)
 * - reachedTarget: Whether target was reached within tolerance
 *
 * OUTPUT:
 * =======
 * CSV file with all metrics for manual analysis in Excel/Python/etc.
 * State file: Saves progress to resume after unexpected shutdown
 *
 * USAGE:
 * ======
 * 1. IMPORTANT: Position turret at CENTER (0°) before starting
 * 2. Run this program to collect data
 * 3. Turret will move sequentially from -180° to +180° (safe, no wire wrapping)
 * 4. Pull CSV from device
 * 5. Analyze in spreadsheet/plotting tool
 * 6. Look for patterns: overshoot, oscillation, slow response, etc.
 * 7. Adjust PID values in TurretConfig based on observations
 * 8. Run again to verify improvements
 *
 * SAFETY:
 * =======
 * - Encoder is reset to zero at start (current position becomes 0°)
 * - Moves sequentially through all positions without returning to zero
 * - Safe range: -180° to +180° (prevents wire wrapping)
 * - Safety checks prevent commands outside this range
 */
@TeleOp(name = "Turret PID Data Collection", group = "Test")
public class TurretPIDTuning extends LinearOpMode {

    // ========== TUNABLE CONSTANTS ==========

    // Runtime limit (20 minutes)
    private static final long MAX_RUNTIME_MS = 20 * 60 * 1000;

    // Target position parameters (testing range)
    private static final double TARGET_MIN_DEGREES = -180.0;
    private static final double TARGET_MAX_DEGREES = 180.0;
    private static final double TARGET_STEP_DEGREES = 15;  // Test every 15 degrees

    // Test parameters
    private static final double POSITION_TOLERANCE_TICKS = 10.0;  // Within ±10 ticks
    private static final long MAX_MOVEMENT_TIME_MS = 5000;  // 5 seconds max per movement
    private static final long STABILITY_WINDOW_MS = 1000;  // Monitor for 1 second after reaching target
    private static final long SAMPLE_INTERVAL_MS = 50;  // Sample every 20ms for detailed data

    // Hardware
    private Turret turret;
    private DcMotorEx turretMotor;
    private KFileWriter dataWriter;
    private OpModeUtilities opModeUtilities;
    private ElapsedTime totalRuntime;
    private ElapsedTime testTimer;

    // State persistence
    private File stateFile;
    private int currentTest = 0;
    private int totalTests = 0;

    // PID values being tested
    private double kp;
    private double ki;
    private double kd;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("=== Turret PID Data Collection ===");
        telemetry.addLine("Initializing...");
        telemetry.update();

        // Initialize
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        turret = Turret.getInstance(opModeUtilities);
        turretMotor = (DcMotorEx) turret.getTurretMotor();
        totalRuntime = new ElapsedTime();
        testTimer = new ElapsedTime();

        // Get PID values from TurretConfig
        kp = TurretConfig.kP;
        ki = TurretConfig.kI;
        kd = TurretConfig.kD;

        // Initialize file writers
        dataWriter = new KFileWriter("TurretPIDData", opModeUtilities);
        initializeStateFile();

        // Write CSV header with all metrics
        dataWriter.writeLine("test,kp,ki,kd,targetDegrees,targetTicks,finalTicks,maxTicks,minTicks,avgTicks," +
            "timeToTarget,overshoot,steadyStateError,oscillationRange,reachedTarget");

        // Calculate total tests
        totalTests = (int) ((TARGET_MAX_DEGREES - TARGET_MIN_DEGREES) / TARGET_STEP_DEGREES) + 1;

        // Load saved state if exists
        loadState();

        // Calculate estimated time
        int estimatedTimeMinutes = (totalTests * 7) / 60;  // ~7 seconds per test

        // Display configuration
        telemetry.addLine("Initialization complete!");
        telemetry.addLine();
        telemetry.addLine("DATA COLLECTION MODE");
        telemetry.addLine("(Not auto-tuning - collecting data for manual analysis)");
        telemetry.addLine();
        telemetry.addData("PID values from TurretConfig", "");
        telemetry.addData("  Kp", "%.6f", kp);
        telemetry.addData("  Ki", "%.6f", kd);
        telemetry.addData("  Kd", "%.6f", kd);
        telemetry.addLine();
        telemetry.addData("Target range", "%.0f° to %.0f° (every %.0f°)",
            TARGET_MIN_DEGREES, TARGET_MAX_DEGREES, TARGET_STEP_DEGREES);
        telemetry.addData("Total tests", "%d", totalTests);
        telemetry.addData("Est. time", "%d min (max 20 min)", estimatedTimeMinutes);
        telemetry.addData("Resuming from", "test %d", currentTest + 1);
        telemetry.addLine();
        telemetry.addLine("SAFETY NOTICE:");
        telemetry.addLine("- Position turret at CENTER (0°) before starting");
        telemetry.addLine("- Will move sequentially from -180° to +180°");
        telemetry.addLine("- No wire wrapping - safe movement pattern");
        telemetry.addLine();
        telemetry.addLine("Press PLAY to start data collection");
        telemetry.update();

        KLog.d("TurretPIDData", String.format("Starting data collection with PID: kp=%.6f, ki=%.6f, kd=%.6f",
            kp, ki, kd));
        KLog.d("TurretPIDData", String.format("Testing %d positions from %.0f° to %.0f° (step: %.0f°)",
            totalTests, TARGET_MIN_DEGREES, TARGET_MAX_DEGREES, TARGET_STEP_DEGREES));

        waitForStart();

        // Reset turret motor ONCE at the start
        // This sets the current position as zero (0 degrees)
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply PID values from TurretConfig
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kp, ki, kd, 0.0);
        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        KLog.d("TurretPIDData", "PID coefficients applied, starting tests...");
        KLog.d("TurretPIDData", "SAFETY: Current position set as 0°. Will move from -180° to +180° sequentially.");

        // Start runtime timer
        totalRuntime.reset();

        // ========== MAIN DATA COLLECTION LOOP ==========
        int testCounter = 0;

        for (double targetDegrees = TARGET_MIN_DEGREES;
             targetDegrees <= TARGET_MAX_DEGREES + 0.01 && opModeIsActive();
             targetDegrees += TARGET_STEP_DEGREES) {

            testCounter++;

            // Skip if we've already completed this test
            if (testCounter <= currentTest) {
                continue;
            }

            // Check runtime limit
            if (totalRuntime.milliseconds() >= MAX_RUNTIME_MS) {
                KLog.d("TurretPIDData", "Runtime limit reached (20 minutes)");
                telemetry.addLine("Runtime limit reached!");
                telemetry.update();
                break;
            }

            if (!opModeIsActive()) break;

            // Display progress
            telemetry.addLine("=== Turret PID Data Collection ===");
            telemetry.addData("Progress", "%d / %d (%.1f%%)",
                testCounter, totalTests,
                100.0 * testCounter / totalTests);
            telemetry.addData("Runtime", "%.1f min / 20 min",
                totalRuntime.milliseconds() / 60000.0);
            telemetry.addLine();
            telemetry.addData("Testing position", "%.0f° (%.2f rad)",
                targetDegrees, Math.toRadians(targetDegrees));
            telemetry.addData("Target ticks", "%.0f",
                Math.toRadians(targetDegrees) * Turret.TICKS_PER_RADIAN);
            telemetry.update();

            KLog.d("TurretPIDData", String.format("========== Test %d/%d: %.0f degrees ==========",
                testCounter, totalTests, targetDegrees));

            // Run test
            double targetRadians = Math.toRadians(targetDegrees);
            TestResult result = runPositionTest(targetRadians);

            // Calculate additional metrics
            double overshoot = 0.0;
            if (result.targetTicks > 0) {
                overshoot = Math.max(0, result.maxTicks - result.targetTicks);
            } else if (result.targetTicks < 0) {
                overshoot = Math.max(0, result.targetTicks - result.minTicks);
            } else {
                overshoot = Math.max(Math.abs(result.maxTicks), Math.abs(result.minTicks));
            }

            double steadyStateError = Math.abs(result.finalTicks - result.targetTicks);
            double oscillationRange = result.maxTicks - result.minTicks;

            // Log to file and save state
            String line = String.format("%d,%.6f,%.6f,%.6f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%b",
                testCounter, kp, ki, kd,
                targetDegrees, result.targetTicks, result.finalTicks,
                result.maxTicks, result.minTicks, result.avgTicks,
                result.timeToTarget, overshoot, steadyStateError, oscillationRange,
                result.reachedTarget);
            dataWriter.writeLine(line);
            saveState(testCounter);

            // Log detailed metrics
            KLog.d("TurretPIDData", String.format("Target: %.0f° (%.1f ticks)", targetDegrees, result.targetTicks));
            KLog.d("TurretPIDData", String.format("Final: %.1f ticks (error: %.1f)", result.finalTicks, steadyStateError));
            KLog.d("TurretPIDData", String.format("Range: [%.1f, %.1f] (oscillation: %.1f)",
                result.minTicks, result.maxTicks, oscillationRange));
            KLog.d("TurretPIDData", String.format("Time: %.0f ms | Overshoot: %.1f | Reached: %s",
                result.timeToTarget, overshoot, result.reachedTarget ? "YES" : "NO"));

            telemetry.addData("Steady-state error", "%.1f ticks", steadyStateError);
            telemetry.addData("Overshoot", "%.1f ticks", overshoot);
            telemetry.addData("Time to target", "%.0f ms", result.timeToTarget);
            telemetry.update();

            sleep(200);  // Brief pause between tests
        }

        // ========== COMPLETION ==========
        // Stop motor
        turretMotor.setPower(0);

        // Close files
        dataWriter.close();
        deleteStateFile();

        // Log completion
        KLog.d("TurretPIDData", "========== DATA COLLECTION COMPLETE ==========");
        KLog.d("TurretPIDData", String.format("Collected %d data points with PID: kp=%.6f ki=%.6f kd=%.6f",
            testCounter, kp, ki, kd));
        KLog.d("TurretPIDData", "Pull CSV file and analyze data to determine better PID values");

        // Display final results
        telemetry.clear();
        telemetry.addLine("=== DATA COLLECTION COMPLETE ===");
        telemetry.addLine();
        telemetry.addData("Tests completed", "%d / %d", testCounter, totalTests);
        telemetry.addData("Total runtime", "%.1f minutes", totalRuntime.milliseconds() / 60000.0);
        telemetry.addLine();
        telemetry.addLine("PID values used:");
        telemetry.addData("Kp", "%.6f", kp);
        telemetry.addData("Ki", "%.6f", ki);
        telemetry.addData("Kd", "%.6f", kd);
        telemetry.addLine();
        telemetry.addLine("NEXT STEPS:");
        telemetry.addLine("1. Pull CSV file from device");
        telemetry.addLine("2. Analyze data (plot, find patterns)");
        telemetry.addLine("3. Adjust PID in TurretConfig");
        telemetry.addLine("4. Run again to verify improvement");
        telemetry.addLine();
        telemetry.addLine("Data saved to:");
        telemetry.addLine("/sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/OdometryLog/");
        telemetry.update();

        // Keep display visible
        while (opModeIsActive()) {
            sleep(100);
        }
    }

    /**
     * Run a single position test at a specific target angle
     * Collects comprehensive data throughout the movement
     */
    private TestResult runPositionTest(double targetRadians) throws InterruptedException {
        TestResult result = new TestResult();

        // Convert target to ticks
        int targetTicks = (int)(targetRadians * Turret.TICKS_PER_RADIAN);
        result.targetTicks = targetTicks;

        // Safety check: ensure target is within safe range
        double targetDegrees = Math.toDegrees(targetRadians);
        if (targetDegrees < TARGET_MIN_DEGREES || targetDegrees > TARGET_MAX_DEGREES) {
            KLog.d("TurretPIDData", String.format(
                "SAFETY: Target %.0f° is outside safe range [%.0f°, %.0f°]. Skipping test.",
                targetDegrees, TARGET_MIN_DEGREES, TARGET_MAX_DEGREES));
            result.reachedTarget = false;
            return result;
        }

        // DO NOT reset encoder here - move from current position to target
        // This ensures smooth sequential movement without returning to zero
        sleep(100);

        // Track all position samples for detailed analysis
        ArrayList<Double> allTicksSamples = new ArrayList<>();
        ArrayList<Double> stabilityTicksSamples = new ArrayList<>();
        boolean reachedTarget = false;
        double timeWhenReachedTarget = 0;

        // Start movement
        testTimer.reset();
        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.8);  // Use reasonable power

        // Monitor movement phase
        while (opModeIsActive() && testTimer.milliseconds() < MAX_MOVEMENT_TIME_MS) {
            int currentTicks = turretMotor.getCurrentPosition();
            allTicksSamples.add((double)currentTicks);

            // Check if reached target
            if (!reachedTarget && Math.abs(currentTicks - targetTicks) <= POSITION_TOLERANCE_TICKS) {
                reachedTarget = true;
                timeWhenReachedTarget = testTimer.milliseconds();
                result.timeToTarget = timeWhenReachedTarget;

                KLog.d("TurretPIDData", String.format(
                    "Reached target %d ticks in %.0fms (current: %d)",
                    targetTicks, result.timeToTarget, currentTicks));
            }

            // If reached target, collect stability window data
            if (reachedTarget) {
                stabilityTicksSamples.add((double)currentTicks);

                // Continue for stability window duration
                if (testTimer.milliseconds() >= timeWhenReachedTarget + STABILITY_WINDOW_MS) {
                    break;
                }
            }

            sleep(SAMPLE_INTERVAL_MS);
        }

        // Record timeout if didn't reach target
        if (!reachedTarget) {
            result.timeToTarget = testTimer.milliseconds();
            KLog.d("TurretPIDData", String.format(
                "Timeout: Did not reach target %d within %dms (final: %d)",
                targetTicks, MAX_MOVEMENT_TIME_MS, turretMotor.getCurrentPosition()));
        }

        // Stop motor
        turretMotor.setPower(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Calculate comprehensive statistics from all samples
        result.finalTicks = turretMotor.getCurrentPosition();
        result.reachedTarget = reachedTarget;

        if (!allTicksSamples.isEmpty()) {
            result.maxTicks = calculateMax(allTicksSamples);
            result.minTicks = calculateMin(allTicksSamples);
            result.avgTicks = calculateAverage(allTicksSamples);
        } else {
            result.maxTicks = result.finalTicks;
            result.minTicks = result.finalTicks;
            result.avgTicks = result.finalTicks;
        }

        return result;
    }

    /**
     * Initialize state file for crash recovery
     */
    private void initializeStateFile() {
        File path = new File(opModeUtilities.getHardwareMap().appContext.getExternalFilesDir(null), "OdometryLog");
        if (!path.exists()) {
            path.mkdirs();
        }
        stateFile = new File(path, "TurretPIDData_state.txt");
    }

    /**
     * Save current state to file
     */
    private void saveState(int testNum) {
        try {
            BufferedWriter writer = new BufferedWriter(new FileWriter(stateFile));
            writer.write(String.format("%d\n", testNum));
            writer.close();
        } catch (IOException e) {
            KLog.d("TurretPIDData", "Failed to save state: " + e.getMessage());
        }
    }

    /**
     * Load saved state from file
     */
    private void loadState() {
        if (!stateFile.exists()) {
            return;
        }

        try {
            BufferedReader reader = new BufferedReader(new FileReader(stateFile));
            String line = reader.readLine();
            reader.close();

            if (line != null) {
                currentTest = Integer.parseInt(line.trim());
                KLog.d("TurretPIDData", String.format("Loaded state: resuming from test %d", currentTest + 1));
            }
        } catch (Exception e) {
            KLog.d("TurretPIDData", "Failed to load state: " + e.getMessage());
            currentTest = 0;
        }
    }

    /**
     * Delete state file after successful completion
     */
    private void deleteStateFile() {
        if (stateFile.exists()) {
            stateFile.delete();
        }
    }

    // Statistics helpers
    private double calculateMax(ArrayList<Double> data) {
        if (data.isEmpty()) return 0.0;
        double max = data.get(0);
        for (double value : data) {
            if (value > max) max = value;
        }
        return max;
    }

    private double calculateMin(ArrayList<Double> data) {
        if (data.isEmpty()) return 0.0;
        double min = data.get(0);
        for (double value : data) {
            if (value < min) min = value;
        }
        return min;
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
        double targetTicks = 0.0;
        double finalTicks = 0.0;
        double maxTicks = 0.0;
        double minTicks = 0.0;
        double avgTicks = 0.0;
        double timeToTarget = 0.0;
        boolean reachedTarget = false;
    }
}
