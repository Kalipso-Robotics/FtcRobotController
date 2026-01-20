package com.kalipsorobotics.test.shooter;

import com.kalipsorobotics.decode.configs.ShooterConfig;
import com.kalipsorobotics.utilities.KFileWriter;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KMotor;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

/**
 * Kp Tuning Data Collection TeleOp
 *
 * This program systematically tests different Kp values for the PIDF controller
 * to find the optimal Kp that provides the best ramp-up performance.
 *
 * Methodology:
 * 1. Loop through different Kp values (around current value of 0.0003)
 * 2. For each Kp value:
 *    - Test target RPS from 15 to 60 in 1.0 RPS increments
 *    - For each target RPS:
 *      a. Use goToRPS() to ramp up motors
 *      b. Measure time to reach target (Ramp_Up_Time)
 *      c. Collect RPS data for 1 second after reaching target
 *      d. Calculate MAX_RPS, MIN_RPS, AVG_RPS
 *      e. Stop motor with BRAKE mode and wait for RPS to reach zero
 *
 * Output file columns: Kp, targetRPS, Ramp_Up_Time, MAX_RPS, MIN_RPS, AVG_RPS
 *
 * Current Configuration:
 * - Kp values tested: 6 (0.0001 to 0.0006 in 0.0001 increments)
 * - RPS tests per Kp: 46 (15 to 60 in 1.0 increments)
 * - Total tests: 276
 * - Estimated time: ~18 minutes
 *
 * Battery Protection & Resume Feature:
 * - Monitors battery voltage before each test
 * - If battery drops below 11.0V, automatically saves progress and stops
 * - Progress is saved after every test completion
 * - On next run, automatically resumes from last completed test
 * - Progress file is cleared when all tests complete successfully
 */
@TeleOp(name = "Kp Tuning Data Collector", group = "Test")
@Disabled
public class KPTuning extends LinearOpMode {

    // Kp test parameters - ADJUST THESE AS NEEDED
    // Testing 6 Kp values around current value (0.0003)
    private static final double KP_START = 0.0001;
    private static final double KP_END = 0.0006;
    private static final double KP_INCREMENT = 0.0001;

    // RPS test parameters
    // Testing RPS in 1.0 increments (46 test points)
    private static final double RPS_START = 15.0;
    private static final double RPS_END = 60.0;
    private static final double RPS_INCREMENT = 1;

    // Timing parameters
    private static final double RPS_TOLERANCE = 0.5; // RPS tolerance for "at target"
    private static final long MAX_RAMP_TIME_MS = 5000; // Max time to wait for ramp-up (5 seconds)
    private static final long DATA_COLLECTION_TIME_MS = 1000; // Collect data for 1 second after reaching target
    private static final long SAMPLE_INTERVAL_MS = 10; // Sample every 50ms
    private static final double ZERO_RPS_THRESHOLD = 0.5; // Consider motor stopped when RPS < 0.5

    // Battery protection
    private static final double MINIMUM_BATTERY_VOLTAGE = 11.0; // Stop testing if battery drops below this
    private static final String PROGRESS_FILE_NAME = "KpTuning_Progress.txt";

    private KMotor shooter1;
    private KMotor shooter2;
    private KFileWriter fileWriter;
    private OpModeUtilities opModeUtilities;
    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("=== Kp Tuning Data Collector ===");
        telemetry.addLine("Initializing...");
        telemetry.update();

        // Initialize hardware
        initializeHardware();
        timer = new ElapsedTime();

        // Initialize file writer
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        // Check for saved progress
        Progress savedProgress = loadProgress();
        boolean isResuming = (savedProgress != null);

        fileWriter = new KFileWriter("KpTuningData", opModeUtilities);

        // Write CSV header only if starting fresh
        if (!isResuming) {
            fileWriter.writeLine("Kp,targetRPS,Ramp_Up_Time,MAX_RPS,MIN_RPS,AVG_RPS");
        }

        // Calculate total tests
        int totalKpTests = (int) ((KP_END - KP_START) / KP_INCREMENT) + 1;
        int totalRpsTests = (int) ((RPS_END - RPS_START) / RPS_INCREMENT) + 1;
        int totalTests = totalKpTests * totalRpsTests;
        int estimatedTimeMinutes = (totalTests * 4) / 60; // Rough estimate: 4 sec per test

        telemetry.addLine("Initialization complete!");
        telemetry.addLine();

        if (isResuming) {
            telemetry.addLine("*** RESUMING FROM SAVED PROGRESS ***");
            telemetry.addData("Last Kp", "%.4f", savedProgress.lastKp);
            telemetry.addData("Last RPS", "%.1f", savedProgress.lastRPS);
            telemetry.addData("Completed Tests", savedProgress.lastTestNumber);
            telemetry.addLine();
        }

        telemetry.addLine("Test Configuration:");
        telemetry.addData("Kp Range", "%.4f to %.4f (step %.4f)", KP_START, KP_END, KP_INCREMENT);
        telemetry.addData("RPS Range", "%.1f to %.1f (step %.1f)", RPS_START, RPS_END, RPS_INCREMENT);
        telemetry.addData("Total Kp Values", totalKpTests);
        telemetry.addData("RPS Tests per Kp", totalRpsTests);
        telemetry.addData("Total Tests", totalTests);
        telemetry.addData("Est. Time", "%d min (~%.1f hrs)", estimatedTimeMinutes, estimatedTimeMinutes / 60.0);
        telemetry.addLine();
        telemetry.addData("Battery Cutoff", "%.1fV", MINIMUM_BATTERY_VOLTAGE);
        telemetry.addLine();
        telemetry.addLine(isResuming ? "Press PLAY to resume" : "Press PLAY to start");
        telemetry.update();

        KLog.d("KPTuning", String.format("Ready to test %d Kp values × %d RPS values = %d total tests",
            totalKpTests, totalRpsTests, totalTests));

        waitForStart();

        // Main data collection loop
        // Resume from saved progress if available
        double currentKp = isResuming ? savedProgress.lastKp : KP_START;
        int overallTestNumber = isResuming ? savedProgress.lastTestNumber + 1 : 1;

        // Calculate kpTestNumber based on currentKp
        int kpTestNumber = (int) ((currentKp - KP_START) / KP_INCREMENT) + 1;

        while (opModeIsActive() && currentKp <= KP_END + 0.00001) { // Small epsilon for float comparison

            // Update Kp value in both motors
            shooter1.getPIDFController().setKp(currentKp);
            shooter2.getPIDFController().setKp(currentKp);

            KLog.d("KPTuning", String.format("========== Testing Kp = %.4f (%d/%d) ==========",
                currentKp, kpTestNumber, totalKpTests));

            // Loop through RPS values for this Kp
            // If resuming and on the same Kp, start from the next RPS after saved progress
            double targetRPS = (isResuming && Math.abs(currentKp - savedProgress.lastKp) < 0.00001)
                ? savedProgress.lastRPS + RPS_INCREMENT
                : RPS_START;

            // Calculate rpsTestNumber based on targetRPS
            int rpsTestNumber = (int) ((targetRPS - RPS_START) / RPS_INCREMENT) + 1;

            while (opModeIsActive() && targetRPS <= RPS_END + 0.01) { // Small epsilon for float comparison

                // Check battery voltage before each test
                double batteryVoltage = getBatteryVoltage();
                if (batteryVoltage < MINIMUM_BATTERY_VOLTAGE) {
                    // Save progress and stop
                    KLog.d("KPTuning", String.format("Battery voltage %.2fV below minimum %.2fV - stopping and saving progress",
                        batteryVoltage, MINIMUM_BATTERY_VOLTAGE));

                    saveProgress(currentKp, targetRPS - RPS_INCREMENT, overallTestNumber - 1);
                    fileWriter.close();

                    shooter1.stop();
                    shooter2.stop();

                    telemetry.addLine("=== LOW BATTERY - STOPPED ===");
                    telemetry.addData("Battery Voltage", "%.2fV", batteryVoltage);
                    telemetry.addData("Minimum Required", "%.2fV", MINIMUM_BATTERY_VOLTAGE);
                    telemetry.addLine();
                    telemetry.addLine("Progress saved!");
                    telemetry.addLine("Recharge battery and run again");
                    telemetry.addLine("to resume from this point.");
                    telemetry.update();

                    while (opModeIsActive()) {
                        sleep(100);
                    }
                    return;
                }

                telemetry.addLine("=== Kp Tuning Data Collection ===");
                telemetry.addData("Overall Progress", "%d / %d", overallTestNumber, totalTests);
                telemetry.addData("Current Kp", "%.4f (%d/%d)", currentKp, kpTestNumber, totalKpTests);
                telemetry.addData("Current RPS Test", "%.1f (%d/%d)", targetRPS, rpsTestNumber, totalRpsTests);
                telemetry.addData("Battery", "%.2fV", batteryVoltage);
                telemetry.addLine();

                // Run the test for this Kp and target RPS
                TestResult result = runSingleTest(currentKp, targetRPS, overallTestNumber, totalTests);

                // Write results to file
                String line = String.format("%.4f,%.1f,%.2f,%.2f,%.2f,%.2f",
                    currentKp, targetRPS, result.rampUpTime, result.maxRPS, result.minRPS, result.avgRPS);
                fileWriter.writeLine(line);

                // Save progress after each test
                saveProgress(currentKp, targetRPS, overallTestNumber);

                KLog.d("KPTuning", String.format(
                    "Test %d/%d | Kp=%.4f RPS=%.1f | RampUp=%.2fms MAX=%.2f MIN=%.2f AVG=%.2f",
                    overallTestNumber, totalTests, currentKp, targetRPS,
                    result.rampUpTime, result.maxRPS, result.minRPS, result.avgRPS));

                // Move to next RPS value
                targetRPS += RPS_INCREMENT;
                rpsTestNumber++;
                overallTestNumber++;

                // After first iteration, we're no longer resuming
                if (isResuming) {
                    isResuming = false;
                }
            }

            // Move to next Kp value
            currentKp += KP_INCREMENT;
            kpTestNumber++;
        }

        // Stop motors
        shooter1.stop();
        shooter2.stop();

        // Close file
        fileWriter.close();

        // Clear progress file since we completed successfully
        clearProgress();

        telemetry.addLine("=== DATA COLLECTION COMPLETE ===");
        telemetry.addLine();
        telemetry.addLine("File saved to:");
        telemetry.addLine("/sdcard/Android/data/");
        telemetry.addLine("com.qualcomm.ftcrobotcontroller/");
        telemetry.addLine("files/OdometryLog/");
        telemetry.addLine();
        telemetry.addLine("Pull file using:");
        telemetry.addLine("adb pull /sdcard/Android/data/");
        telemetry.addLine("com.qualcomm.ftcrobotcontroller/");
        telemetry.addLine("files/OdometryLog/ ~/");
        telemetry.update();

        KLog.d("KPTuning", "Data collection complete. File closed.");

        // Keep telemetry visible
        while (opModeIsActive()) {
            sleep(100);
        }
    }

    /**
     * Run a single test for a given Kp and target RPS
     */
    private TestResult runSingleTest(double kp, double targetRPS, int testNum, int totalTests) throws InterruptedException {
        TestResult result = new TestResult();

        // Reset motors to ensure clean start
        shooter1.resetPID();
        shooter2.resetPID();

        telemetry.addLine("Status: Ramping up to " + String.format("%.1f", targetRPS) + " RPS...");
        telemetry.update();

        // Start ramping up
        timer.reset();
        shooter1.goToRPS(targetRPS);
        shooter2.goToRPS(targetRPS);

        // Wait until target RPS is reached or timeout
        boolean reachedTarget = false;
        while (opModeIsActive() && timer.milliseconds() < MAX_RAMP_TIME_MS) {
            // Continue calling goToRPS to update PID
            shooter1.goToRPS(targetRPS);
            shooter2.goToRPS(targetRPS);

            double currentRPS = shooter1.getRPS();

            // Check if we've reached target
            if (Math.abs(currentRPS - targetRPS) <= RPS_TOLERANCE) {
                reachedTarget = true;
                result.rampUpTime = timer.milliseconds();
                KLog.d("KPTuning", String.format("Reached target! Time: %.2fms, RPS: %.2f",
                    result.rampUpTime, currentRPS));
                break;
            }

            telemetry.addLine("Status: Ramping up...");
            telemetry.addData("Target RPS", "%.1f", targetRPS);
            telemetry.addData("Current RPS", "%.2f", currentRPS);
            telemetry.addData("Time Elapsed", "%.0f ms", timer.milliseconds());
            telemetry.update();

            sleep(SAMPLE_INTERVAL_MS);
        }

        // If didn't reach target, record the time taken anyway
        if (!reachedTarget) {
            result.rampUpTime = timer.milliseconds();
            KLog.d("KPTuning", String.format("Did not reach target RPS %.1f within %dms",
                targetRPS, MAX_RAMP_TIME_MS));
        }

        telemetry.addLine("Status: Collecting stability data...");
        telemetry.update();

        // Collect RPS data for 1 second after reaching target
        ArrayList<Double> rpsData = new ArrayList<>();
        long startTime = System.currentTimeMillis();
        long endTime = startTime + DATA_COLLECTION_TIME_MS;

        while (System.currentTimeMillis() < endTime && opModeIsActive()) {
            // Continue calling goToRPS to maintain target
            shooter1.goToRPS(targetRPS);
            shooter2.goToRPS(targetRPS);

            double currentRPS = shooter1.getRPS();
            rpsData.add(currentRPS);

            telemetry.addLine("Status: Collecting stability data...");
            telemetry.addData("Samples", rpsData.size());
            telemetry.addData("Current RPS", "%.2f", currentRPS);
            telemetry.update();

            sleep(SAMPLE_INTERVAL_MS);
        }

        // Calculate statistics
        result.maxRPS = calculateMax(rpsData);
        result.minRPS = calculateMin(rpsData);
        result.avgRPS = calculateAverage(rpsData);

        telemetry.addLine("Status: Stopping motor...");
        telemetry.update();

        // Stop motor with BRAKE mode for faster stopping
        shooter1.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter1.stop();
        shooter2.stop();

        // Wait for RPS to reach zero
        while (opModeIsActive()) {
            double currentRPS = shooter1.getRPS();
            if (currentRPS < ZERO_RPS_THRESHOLD) {
                break;
            }

            telemetry.addLine("Status: Waiting for motor to stop...");
            telemetry.addData("Current RPS", "%.2f", currentRPS);
            telemetry.update();

            sleep(SAMPLE_INTERVAL_MS);
        }

        // Return to FLOAT mode for next test
        shooter1.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Brief pause before next test
        sleep(200);

        telemetry.addLine("Status: Test complete!");
        telemetry.addData("Ramp Up Time", "%.2f ms", result.rampUpTime);
        telemetry.addData("Max RPS", "%.2f", result.maxRPS);
        telemetry.addData("Min RPS", "%.2f", result.minRPS);
        telemetry.addData("Avg RPS", "%.2f", result.avgRPS);
        telemetry.update();

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
            ShooterConfig.kp_accel, ShooterConfig.ki_accel, ShooterConfig.kd_accel, ShooterConfig.kf_accel);

        // Initialize shooter2
        DcMotor motor2 = hardwareMap.dcMotor.get("shooter2");
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter2 = new KMotor(motor2,
            ShooterConfig.kp_accel, ShooterConfig.ki_accel, ShooterConfig.kd_accel, ShooterConfig.kf_accel);

        KLog.d("KPTuning", "Motors initialized successfully");
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
     * Get battery voltage from voltage sensor
     */
    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    /**
     * Save current progress to file for resuming later
     */
    private void saveProgress(double currentKp, double currentRPS, int testNumber) {
        try {
            File path = new File(opModeUtilities.getHardwareMap().appContext.getExternalFilesDir(null), "OdometryLog");
            if (!path.exists()) {
                path.mkdirs();
            }
            File progressFile = new File(path, PROGRESS_FILE_NAME);
            BufferedWriter writer = new BufferedWriter(new FileWriter(progressFile));
            writer.write(String.format("%.4f,%.1f,%d", currentKp, currentRPS, testNumber));
            writer.newLine();
            writer.close();
            KLog.d("KPTuning", String.format("Progress saved: Kp=%.4f, RPS=%.1f, Test#=%d",
                currentKp, currentRPS, testNumber));
        } catch (IOException e) {
            KLog.e("KPTuning", "Failed to save progress", e);
        }
    }

    /**
     * Load saved progress from file
     * Returns null if no saved progress exists
     */
    private Progress loadProgress() {
        try {
            File path = new File(opModeUtilities.getHardwareMap().appContext.getExternalFilesDir(null), "OdometryLog");
            File progressFile = new File(path, PROGRESS_FILE_NAME);

            if (!progressFile.exists()) {
                KLog.d("KPTuning", "No saved progress found - starting fresh");
                return null;
            }

            BufferedReader reader = new BufferedReader(new FileReader(progressFile));
            String line = reader.readLine();
            reader.close();

            if (line == null || line.isEmpty()) {
                return null;
            }

            String[] parts = line.split(",");
            if (parts.length != 3) {
                KLog.d("KPTuning", "Invalid progress file format - starting fresh");
                return null;
            }

            Progress progress = new Progress();
            progress.lastKp = Double.parseDouble(parts[0]);
            progress.lastRPS = Double.parseDouble(parts[1]);
            progress.lastTestNumber = Integer.parseInt(parts[2]);

            KLog.d("KPTuning", String.format("Progress loaded: Kp=%.4f, RPS=%.1f, Test#=%d",
                progress.lastKp, progress.lastRPS, progress.lastTestNumber));

            return progress;
        } catch (IOException | NumberFormatException e) {
            KLog.e("KPTuning", "Failed to load progress - starting fresh", e);
            return null;
        }
    }

    /**
     * Delete saved progress file (call when test completes successfully)
     */
    private void clearProgress() {
        try {
            File path = new File(opModeUtilities.getHardwareMap().appContext.getExternalFilesDir(null), "OdometryLog");
            File progressFile = new File(path, PROGRESS_FILE_NAME);
            if (progressFile.exists()) {
                progressFile.delete();
                KLog.d("KPTuning", "Progress file cleared");
            }
        } catch (Exception e) {
            KLog.e("KPTuning", "Failed to clear progress file", e);
        }
    }

    /**
     * Simple class to hold saved progress
     */
    private static class Progress {
        double lastKp = 0.0;
        double lastRPS = 0.0;
        int lastTestNumber = 0;
    }

    /**
     * Simple class to hold test results
     */
    private static class TestResult {
        double rampUpTime = 0.0;
        double maxRPS = 0.0;
        double minRPS = 0.0;
        double avgRPS = 0.0;
    }
}
