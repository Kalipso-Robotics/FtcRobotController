package com.kalipsorobotics.test.turret;

import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KMotor;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Locale;

/**
 * TurretKsTuner - Finds the minimum static friction power (kS) for the turret motor
 *
 * This program systematically tests different power values to find the minimum power
 * needed to overcome static friction and start moving the turret.
 *
 * Controls:
 * - A button: Start/Resume tuning
 * - B button: Stop tuning
 * - X button: Save and exit
 *
 * Data is automatically saved periodically to prevent loss if program stops.
 */
//@TeleOp(name = "Turret kS Tuner", group = "Tuning")
public class TurretKsTuner extends LinearOpMode {

    private OpModeUtilities opModeUtilities;
    private Turret turret;
    private KMotor turretMotor;

    // Tuning parameters
    private static final double MIN_POWER = 0.0;
    private static final double MAX_POWER = 0.5;
    private static final double POWER_INCREMENT = 0.005;  // Small increments for precision
    private static final double MIN_VELOCITY_THRESHOLD = 0.01;  // Minimum RPS to consider movement
    private static final int TEST_DURATION_MS = 300;  // How long to apply power
    private static final int VELOCITY_SAMPLE_DELAY_MS = 150;  // Time to wait before sampling velocity
    private static final int SETTLE_TIME_MS = 500;  // Time to wait for motor to settle
    private static final int TRIALS_PER_POWER = 3;  // Number of times to test each power

    // Data storage
    private final ArrayList<TestResult> results = new ArrayList<>();
    private File dataFile;
    private boolean isRunning = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        turret = Turret.getInstance(opModeUtilities);
        turretMotor = turret.getTurretMotor();

        // Setup data file
        setupDataFile();

        telemetry.addLine("Turret kS Tuner Ready");
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("  A - Start/Resume tuning");
        telemetry.addLine("  B - Stop tuning");
        telemetry.addLine("  X - Save and exit");
        telemetry.addLine("");
        telemetry.addData("Data file", dataFile.getAbsolutePath());
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                // Control logic
                if (gamepad1.a && !isRunning) {
                    isRunning = true;
                    runTuningSequence();
                    isRunning = false;
                }

                if (gamepad1.b && isRunning) {
                    isRunning = false;
                    telemetry.addLine("Tuning stopped by user");
                    telemetry.update();
                }

                if (gamepad1.x) {
                    saveResults();
                    telemetry.addLine("Data saved. Exiting...");
                    telemetry.update();
                    sleep(1000);
                    break;
                }

                // Display status
                if (!isRunning) {
                    telemetry.addLine("Press A to start tuning");
                    telemetry.addData("Tests completed", results.size());
                    if (!results.isEmpty()) {
                        double avgMinPower = calculateAverageMinimumPower();
                        telemetry.addData("Current avg minimum power", String.format("%.4f", avgMinPower));
                    }
                    telemetry.update();
                }

                sleep(50);
            }
        } finally {
            // Ensure motor is stopped and data is saved
            turretMotor.stop();
            saveResults();
            telemetry.addLine("Cleanup complete");
            telemetry.update();
        }
    }

    /**
     * Run the main tuning sequence
     */
    private void runTuningSequence() {
        telemetry.addLine("Starting tuning sequence...");
        telemetry.update();

        double currentPower = MIN_POWER;
        boolean foundMinimum = false;

        while (opModeIsActive() && isRunning && currentPower <= MAX_POWER) {
            telemetry.addData("Testing power", String.format("%.4f", currentPower));
            telemetry.update();

            // Test this power level multiple times
            int successCount = 0;
            ArrayList<Double> velocities = new ArrayList<>();

            for (int trial = 0; trial < TRIALS_PER_POWER && isRunning; trial++) {
                telemetry.addData("Testing power", String.format("%.4f", currentPower));
                telemetry.addData("Trial", (trial + 1) + "/" + TRIALS_PER_POWER);
                telemetry.update();

                // Test positive direction
                double velocity = testPower(currentPower);
                if (Math.abs(velocity) >= MIN_VELOCITY_THRESHOLD) {
                    successCount++;
                    velocities.add(Math.abs(velocity));
                }

                if (!isRunning) break;

                // Test negative direction
                velocity = testPower(-currentPower);
                if (Math.abs(velocity) >= MIN_VELOCITY_THRESHOLD) {
                    successCount++;
                    velocities.add(Math.abs(velocity));
                }

                sleep(SETTLE_TIME_MS);
            }

            // Record results
            boolean moved = successCount > 0;
            TestResult result = new TestResult(
                currentPower,
                moved,
                successCount,
                TRIALS_PER_POWER * 2,  // Tested both directions
                velocities
            );
            results.add(result);

            // Log result
            KLog.d("TurretKsTuner", String.format("Power: %.4f, Moved: %s, Success: %d/%d",
                currentPower, moved, successCount, TRIALS_PER_POWER * 2));

            // Save periodically (every 10 tests)
            if (results.size() % 10 == 0) {
                saveResults();
            }

            // If we found movement, mark it
            if (moved && !foundMinimum) {
                foundMinimum = true;
                telemetry.addLine("Found minimum power!");
                telemetry.addData("Minimum power", String.format("%.4f", currentPower));
                telemetry.update();
                sleep(1000);
            }

            currentPower += POWER_INCREMENT;
        }

        // Final save
        saveResults();

        // Display results
        displayFinalResults();
    }

    /**
     * Test a specific power value
     * @param power The power to test
     * @return The average velocity in RPS (rotations per second)
     */
    private double testPower(double power) {
        // Apply power
        turretMotor.setPower(power);

        // Wait for velocity to stabilize
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < VELOCITY_SAMPLE_DELAY_MS && opModeIsActive()) {
            sleep(10);
        }

        // Sample velocity multiple times and average
        double velocitySum = 0;
        int samples = 0;
        timer.reset();
        while (timer.milliseconds() < (TEST_DURATION_MS - VELOCITY_SAMPLE_DELAY_MS) && opModeIsActive()) {
            double rps = turretMotor.getRPS();
            velocitySum += Math.abs(rps);
            samples++;
            sleep(20);
        }

        // Stop motor
        turretMotor.stop();

        // Calculate average velocity
        double avgVelocity = samples > 0 ? velocitySum / samples : 0.0;

        KLog.d("TurretKsTuner", String.format("Power: %.4f, Avg Velocity: %.4f RPS, Samples: %d",
            power, avgVelocity, samples));

        return avgVelocity;
    }

    /**
     * Calculate the average minimum power from successful tests
     */
    private double calculateAverageMinimumPower() {
        ArrayList<Double> minimumPowers = new ArrayList<>();

        for (TestResult result : results) {
            if (result.moved) {
                minimumPowers.add(result.power);
            }
        }

        if (minimumPowers.isEmpty()) {
            return 0.0;
        }

        // Find the lowest few powers that worked
        double sum = 0;
        int count = 0;
        for (int i = 0; i < Math.min(5, minimumPowers.size()); i++) {
            sum += minimumPowers.get(i);
            count++;
        }

        return count > 0 ? sum / count : 0.0;
    }

    /**
     * Display final results
     */
    private void displayFinalResults() {
        telemetry.clear();
        telemetry.addLine("=== TUNING COMPLETE ===");
        telemetry.addLine("");
        telemetry.addData("Total tests", results.size());

        double avgMinPower = calculateAverageMinimumPower();
        telemetry.addData("Average minimum kS", String.format("%.4f", avgMinPower));

        telemetry.addLine("");
        telemetry.addLine("First few successful powers:");
        int count = 0;
        for (TestResult result : results) {
            if (result.moved && count < 5) {
                telemetry.addData(String.format("  %.4f", result.power),
                    String.format("Success: %d/%d", result.successCount, result.totalTrials));
                count++;
            }
        }

        telemetry.addLine("");
        telemetry.addData("Data saved to", dataFile.getName());
        telemetry.addLine("Press X to exit");
        telemetry.update();
    }

    /**
     * Setup the data file
     */
    private void setupDataFile() {
        // Use the FTC data directory
        File dataDir = new File("/sdcard/FIRST/data");
        if (!dataDir.exists()) {
            dataDir.mkdirs();
        }

        // Create timestamped filename
        SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US);
        String timestamp = sdf.format(new Date());
        dataFile = new File(dataDir, "turret_ks_tuning_" + timestamp + ".csv");

        try {
            // Create file and write header
            BufferedWriter writer = new BufferedWriter(new FileWriter(dataFile, false));
            writer.write("Power,Moved,SuccessCount,TotalTrials,AvgVelocity_RPS\n");
            writer.close();
        } catch (IOException e) {
            KLog.e("TurretKsTuner", "Failed to create data file: " + e.getMessage());
            telemetry.addLine("ERROR: Could not create data file!");
        }
    }

    /**
     * Save results to file
     */
    private void saveResults() {
        try {
            BufferedWriter writer = new BufferedWriter(new FileWriter(dataFile, false));
            writer.write("Power,Moved,SuccessCount,TotalTrials,AvgVelocity_RPS\n");

            for (TestResult result : results) {
                writer.write(String.format(Locale.US, "%.4f,%s,%d,%d,%.4f\n",
                    result.power,
                    result.moved,
                    result.successCount,
                    result.totalTrials,
                    result.getAverageVelocity()));
            }

            writer.close();
            KLog.d("TurretKsTuner", "Results saved to " + dataFile.getAbsolutePath());
        } catch (IOException e) {
            KLog.e("TurretKsTuner", "Failed to save results: " + e.getMessage());
            telemetry.addLine("ERROR: Could not save data!");
        }
    }

    /**
     * Data class to store test results
     */
    private static class TestResult {
        double power;
        boolean moved;
        int successCount;
        int totalTrials;
        ArrayList<Double> velocities;

        TestResult(double power, boolean moved, int successCount, int totalTrials, ArrayList<Double> velocities) {
            this.power = power;
            this.moved = moved;
            this.successCount = successCount;
            this.totalTrials = totalTrials;
            this.velocities = new ArrayList<>(velocities);
        }

        double getAverageVelocity() {
            if (velocities.isEmpty()) return 0.0;
            double sum = 0;
            for (double velocity : velocities) {
                sum += velocity;
            }
            return sum / velocities.size();
        }
    }
}
