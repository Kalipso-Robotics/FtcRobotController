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
 * PID Iterative Learning TeleOp
 *
 * This program uses an iterative learning/hill-climbing approach to find optimal
 * PID values (kp, ki, kd) for the shooter PIDF controller.
 *
 * LEARNING STRATEGY:
 * ==================
 * Uses a simple hill-climbing algorithm with adaptive perturbations:
 *
 * 1. Start with initial PID values from ShooterConfig
 * 2. For each iteration:
 *    a. Test current PID values across all target RPS values (15-60)
 *    b. Measure performance: ramp-up time, overshoot, undershoot
 *    c. Calculate "cost" (lower = better performance)
 *    d. If cost improved, continue with small random perturbations
 *    e. If cost worsened, explore more aggressively
 * 3. Track best PID values found across all iterations
 *
 * COST FUNCTION:
 * ==============
 * Lower cost = better performance
 * Factors weighted and summed:
 *   - Ramp-up time (normalized by target RPS)
 *   - Overshoot (deviation above target)
 *   - Undershoot (deviation below target)
 *   - Instability (oscillation range during stability window)
 *
 * TUNABLE CONSTANTS:
 * ==================
 * - MAX_ITERATIONS: Number of learning iterations (default: 20)
 * - RPS_START/END/STEP: Range of RPS to test (default: 15-60 in steps of 5)
 * - PERTURBATION_RATE: How much to adjust PID each iteration (default: 10%)
 * - COST_WEIGHTS: Adjust to prioritize different performance aspects
 * - LEARNING_RATE: Controls exploration vs exploitation trade-off
 *
 * OUTPUT:
 * =======
 * CSV file with columns: Iteration,targetRPS,kp,ki,kd,avgRampUpTime,avgMaxRPS,avgMinRPS
 */
@TeleOp(name = "PID Iterative Learning", group = "Test")
public class PIDIterativeLearning extends LinearOpMode {

    // ========== TUNABLE CONSTANTS ==========

    // Learning parameters
    private static final int MAX_ITERATIONS = 20;  // Number of learning iterations
    private static final double PERTURBATION_RATE = 0.15;  // 15% max adjustment per iteration
    private static final double LEARNING_RATE = 0.5;  // Controls exploration magnitude (0.1-1.0)

    // Target RPS test range
    private static final double RPS_START = 15.0;
    private static final double RPS_END = 60.0;
    private static final double RPS_STEP = 5.0;  // Test every 5 RPS

    // Test parameters
    private static final double RPS_TOLERANCE = 0.5;  // Consider "at target" when within ±0.5 RPS
    private static final long MAX_RAMP_TIME_MS = 5000;  // Max time to wait for ramp-up (5 seconds)
    private static final long STABILITY_WINDOW_MS = 500;  // Monitor stability for 500ms after reaching target
    private static final long SAMPLE_INTERVAL_MS = 50;  // Sample RPS every 50ms

    // Cost function weights (adjust to prioritize different factors)
    // Higher weight = more important to minimize
    private static final double WEIGHT_RAMP_TIME = 1.0;  // Fast ramp-up
    private static final double WEIGHT_OVERSHOOT = 2.0;  // Avoid overshoot (most important)
    private static final double WEIGHT_UNDERSHOOT = 1.5;  // Avoid undershoot
    private static final double WEIGHT_INSTABILITY = 1.0;  // Minimize oscillation

    // PID value bounds (prevents unstable values)
    private static final double KP_MIN = 0.00001;
    private static final double KP_MAX = 0.01;
    private static final double KI_MIN = 0.0;
    private static final double KI_MAX = 0.001;
    private static final double KD_MIN = 0.0;
    private static final double KD_MAX = 0.001;

    // Hardware
    private KMotor shooter1;
    private KMotor shooter2;
    private KFileWriter fileWriter;
    private OpModeUtilities opModeUtilities;
    private ElapsedTime timer;

    // Current PID values (updated each iteration)
    private double currentKp;
    private double currentKi;
    private double currentKd;
    private double currentKf;  // Feedforward from ShooterConfig (not tuned)

    // Tracking best performance
    private double bestCost = Double.POSITIVE_INFINITY;
    private double bestKp;
    private double bestKi;
    private double bestKd;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("=== PID Iterative Learning ===");
        telemetry.addLine("Initializing...");
        telemetry.update();

        // Initialize hardware
        initializeHardware();
        timer = new ElapsedTime();

        // Initialize file writer
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        fileWriter = new KFileWriter("PIDIterativeLearningData", opModeUtilities);

        // Write CSV header
        fileWriter.writeLine("Iteration,targetRPS,kp,ki,kd,avgRampUpTime,avgMaxRPS,avgMinRPS");

        // Initialize PID values from ShooterConfig
        currentKp = ShooterConfig.kp;
        currentKi = ShooterConfig.ki;
        currentKd = ShooterConfig.kd;
        currentKf = ShooterConfig.kf;

        // Initialize best values
        bestKp = currentKp;
        bestKi = currentKi;
        bestKd = currentKd;

        // Calculate total tests
        int totalRpsTests = (int) ((RPS_END - RPS_START) / RPS_STEP) + 1;
        int totalTests = MAX_ITERATIONS * totalRpsTests;
        int estimatedTimeMinutes = (totalTests * 6) / 60;  // ~6 seconds per test

        telemetry.addLine("Initialization complete!");
        telemetry.addLine();
        telemetry.addLine("Learning Configuration:");
        telemetry.addData("Max Iterations", MAX_ITERATIONS);
        telemetry.addData("RPS Range", "%.1f to %.1f (step %.1f)", RPS_START, RPS_END, RPS_STEP);
        telemetry.addData("Tests per Iteration", totalRpsTests);
        telemetry.addData("Total Tests", totalTests);
        telemetry.addData("Est. Time", "%d min", estimatedTimeMinutes);
        telemetry.addLine();
        telemetry.addLine("Initial PID Values:");
        telemetry.addData("Kp", "%.6f", currentKp);
        telemetry.addData("Ki", "%.6f", currentKi);
        telemetry.addData("Kd", "%.6f", currentKd);
        telemetry.addData("Kf (fixed)", "%.6f", currentKf);
        telemetry.addLine();
        telemetry.addLine("Press PLAY to start learning");
        telemetry.update();

        KLog.d("PIDIterativeLearning", String.format("Starting with PID: kp=%.6f, ki=%.6f, kd=%.6f, kf=%.6f",
            currentKp, currentKi, currentKd, currentKf));

        waitForStart();

        // ========== MAIN LEARNING LOOP ==========
        for (int iteration = 1; iteration <= MAX_ITERATIONS && opModeIsActive(); iteration++) {

            telemetry.addLine("=== PID Iterative Learning ===");
            telemetry.addData("Iteration", "%d / %d", iteration, MAX_ITERATIONS);
            telemetry.addLine();
            telemetry.addLine("Current PID values:");
            telemetry.addData("Kp", "%.6f", currentKp);
            telemetry.addData("Ki", "%.6f", currentKi);
            telemetry.addData("Kd", "%.6f", currentKd);
            telemetry.addLine();
            telemetry.addLine("Best so far:");
            telemetry.addData("Best Cost", "%.2f", bestCost);
            telemetry.addData("Best Kp", "%.6f", bestKp);
            telemetry.update();

            KLog.d("PIDIterativeLearning", String.format("========== Iteration %d/%d ==========",
                iteration, MAX_ITERATIONS));
            KLog.d("PIDIterativeLearning", String.format("Testing PID: kp=%.6f, ki=%.6f, kd=%.6f",
                currentKp, currentKi, currentKd));

            // Apply current PID values to motors
            shooter1.getPIDFController().setKp(currentKp);
            shooter1.getPIDFController().setKi(currentKi);
            shooter1.getPIDFController().setKd(currentKd);

            shooter2.getPIDFController().setKp(currentKp);
            shooter2.getPIDFController().setKi(currentKi);
            shooter2.getPIDFController().setKd(currentKd);

            // Test current PID values across all target RPS
            double totalCost = 0.0;
            int testCount = 0;

            for (double targetRPS = RPS_START; targetRPS <= RPS_END + 0.01 && opModeIsActive(); targetRPS += RPS_STEP) {

                telemetry.addData("Testing RPS", "%.1f", targetRPS);
                telemetry.update();

                // Run single test
                TestResult result = runSingleTest(targetRPS, iteration, testCount + 1);

                // Calculate cost for this test
                double cost = calculateCost(result, targetRPS);
                totalCost += cost;
                testCount++;

                // Log results to file
                String line = String.format("%d,%.1f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f",
                    iteration, targetRPS, currentKp, currentKi, currentKd,
                    result.rampUpTime, result.maxRPS, result.minRPS);
                fileWriter.writeLine(line);

                KLog.d("PIDIterativeLearning", String.format(
                    "Iter %d | RPS %.1f | Ramp %.0fms | Max %.2f | Min %.2f | Cost %.2f",
                    iteration, targetRPS, result.rampUpTime, result.maxRPS, result.minRPS, cost));

                telemetry.addData("Last Test Cost", "%.2f", cost);
                telemetry.update();
            }

            // Calculate average cost for this iteration
            double avgCost = totalCost / testCount;

            telemetry.addData("Iteration Avg Cost", "%.2f", avgCost);
            telemetry.update();

            KLog.d("PIDIterativeLearning", String.format("Iteration %d complete | Avg Cost: %.2f",
                iteration, avgCost));

            // Check if this is the best performance so far
            if (avgCost < bestCost) {
                bestCost = avgCost;
                bestKp = currentKp;
                bestKi = currentKi;
                bestKd = currentKd;

                KLog.d("PIDIterativeLearning", String.format(
                    "*** NEW BEST! Cost: %.2f | PID: kp=%.6f ki=%.6f kd=%.6f",
                    bestCost, bestKp, bestKi, bestKd));

                telemetry.addLine("*** NEW BEST FOUND! ***");
                telemetry.update();
                sleep(1000);  // Show message
            }

            // Update PID values for next iteration using hill-climbing
            if (iteration < MAX_ITERATIONS) {
                updatePIDValues(avgCost);
            }

            sleep(300);  // Brief pause between iterations
        }

        // Stop motors
        shooter1.stop();
        shooter2.stop();

        // Close file
        fileWriter.close();

        // Display final results
        telemetry.addLine("=== LEARNING COMPLETE ===");
        telemetry.addLine();
        telemetry.addLine("Best PID Values Found:");
        telemetry.addData("Kp", "%.6f", bestKp);
        telemetry.addData("Ki", "%.6f", bestKi);
        telemetry.addData("Kd", "%.6f", bestKd);
        telemetry.addData("Kf", "%.6f (unchanged)", currentKf);
        telemetry.addData("Best Cost", "%.2f", bestCost);
        telemetry.addLine();
        telemetry.addLine("Update ShooterConfig.java with:");
        telemetry.addData("kp", "%.6f", bestKp);
        telemetry.addData("ki", "%.6f", bestKi);
        telemetry.addData("kd", "%.6f", bestKd);
        telemetry.addLine();
        telemetry.addLine("File saved to:");
        telemetry.addLine("/sdcard/Android/data/");
        telemetry.addLine("com.qualcomm.ftcrobotcontroller/");
        telemetry.addLine("files/OdometryLog/");
        telemetry.update();

        KLog.d("PIDIterativeLearning", String.format(
            "Learning complete! Best PID: kp=%.6f ki=%.6f kd=%.6f (cost: %.2f)",
            bestKp, bestKi, bestKd, bestCost));

        // Keep telemetry visible
        while (opModeIsActive()) {
            sleep(100);
        }
    }

    /**
     * Run a single test at a specific target RPS
     *
     * Process:
     * 1. Reset PID state
     * 2. Command goToRPS and measure time to reach target
     * 3. Once at target, monitor for 500ms to measure stability
     * 4. Stop motors and wait for them to spin down
     *
     * @param targetRPS The target RPS to test
     * @param iteration Current iteration number
     * @param testNum Test number within iteration
     * @return TestResult containing ramp time, max RPS, min RPS
     */
    private TestResult runSingleTest(double targetRPS, int iteration, int testNum) throws InterruptedException {
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

            // Check if we've reached target (within ±0.5 RPS)
            if (Math.abs(currentRPS - targetRPS) <= RPS_TOLERANCE) {
                reachedTarget = true;
                result.rampUpTime = timer.milliseconds();
                KLog.d("PIDIterativeLearning", String.format(
                    "Reached target %.1f RPS in %.0fms (actual: %.2f)",
                    targetRPS, result.rampUpTime, currentRPS));
                break;
            }

            sleep(SAMPLE_INTERVAL_MS);
        }

        // If didn't reach target within timeout, record the timeout
        if (!reachedTarget) {
            result.rampUpTime = timer.milliseconds();
            KLog.d("PIDIterativeLearning", String.format(
                "Timeout: Did not reach target RPS %.1f within %dms",
                targetRPS, MAX_RAMP_TIME_MS));
        }

        // ========== STABILITY MONITORING PHASE ==========
        // Collect RPS data for 500ms after reaching target
        // This measures overshoot, undershoot, and oscillation
        ArrayList<Double> rpsData = new ArrayList<>();
        long startTime = System.currentTimeMillis();
        long endTime = startTime + STABILITY_WINDOW_MS;

        while (System.currentTimeMillis() < endTime && opModeIsActive()) {
            // Continue calling goToRPS to maintain target
            shooter1.goToRPS(targetRPS);
            shooter2.goToRPS(targetRPS);

            double currentRPS = shooter1.getRPS();
            rpsData.add(currentRPS);

            sleep(SAMPLE_INTERVAL_MS);
        }

        // Calculate max and min RPS during stability window
        if (!rpsData.isEmpty()) {
            result.maxRPS = calculateMax(rpsData);
            result.minRPS = calculateMin(rpsData);
        } else {
            // No data collected - use poor defaults
            result.maxRPS = 0.0;
            result.minRPS = 0.0;
        }

        // ========== MOTOR STOP & COOLDOWN ==========
        // Stop motors with brake mode for faster deceleration
        shooter1.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter1.stop();
        shooter2.stop();

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
     * Calculate cost for a test result
     *
     * Lower cost = better performance
     *
     * Cost components:
     * 1. Ramp-up time (normalized by target RPS to account for higher speeds taking longer)
     * 2. Overshoot (how much max RPS exceeds target - bad for accuracy)
     * 3. Undershoot (how much min RPS falls below target - indicates instability)
     * 4. Instability (range of oscillation during stability window)
     *
     * @param result Test result data
     * @param targetRPS Target RPS for this test
     * @return Cost value (lower is better)
     */
    private double calculateCost(TestResult result, double targetRPS) {
        // Normalize ramp-up time by target RPS
        // Higher RPS naturally takes longer to reach, so we normalize
        double normalizedRampTime = result.rampUpTime / targetRPS;

        // Calculate overshoot (exceeding target is bad)
        double overshoot = Math.max(0, result.maxRPS - targetRPS);

        // Calculate undershoot (falling below target is bad)
        double undershoot = Math.max(0, targetRPS - result.minRPS);

        // Calculate instability (oscillation range)
        double instability = result.maxRPS - result.minRPS;

        // Combine into total cost with weights
        double cost = WEIGHT_RAMP_TIME * normalizedRampTime
                    + WEIGHT_OVERSHOOT * overshoot
                    + WEIGHT_UNDERSHOOT * undershoot
                    + WEIGHT_INSTABILITY * instability;

        return cost;
    }

    /**
     * Update PID values for next iteration using hill-climbing strategy
     *
     * STRATEGY:
     * - If current cost is better than best so far:
     *   We're moving in a good direction → small exploratory perturbations
     *
     * - If current cost is worse than best so far:
     *   We went the wrong way → larger exploration to escape local minimum
     *
     * - Perturbations are random but constrained by PERTURBATION_RATE
     * - Values are clamped to safe ranges to prevent instability
     *
     * This is a gradient-free optimization approach that works well
     * when the cost function is noisy or non-differentiable.
     *
     * @param currentCost Cost of current iteration
     */
    private void updatePIDValues(double currentCost) {
        // Determine if we're improving
        boolean improving = (currentCost < bestCost);

        // Calculate perturbation magnitude based on performance
        double perturbation;
        if (improving) {
            // We're improving - make small exploratory changes
            perturbation = PERTURBATION_RATE * LEARNING_RATE;
        } else {
            // We're not improving - explore more aggressively
            perturbation = PERTURBATION_RATE * LEARNING_RATE * 2.0;
        }

        // Apply random perturbations to each PID value
        // Random value in range [-perturbation, +perturbation]
        double kpChange = (Math.random() - 0.5) * 2.0 * perturbation;
        double kiChange = (Math.random() - 0.5) * 2.0 * perturbation;
        double kdChange = (Math.random() - 0.5) * 2.0 * perturbation;

        // Update values with multiplicative perturbations
        currentKp *= (1.0 + kpChange);
        currentKi *= (1.0 + kiChange);
        currentKd *= (1.0 + kdChange);

        // Clamp values to safe ranges to prevent instability
        currentKp = Math.max(KP_MIN, Math.min(KP_MAX, currentKp));
        currentKi = Math.max(KI_MIN, Math.min(KI_MAX, currentKi));
        currentKd = Math.max(KD_MIN, Math.min(KD_MAX, currentKd));

        KLog.d("PIDIterativeLearning", String.format(
            "Updated PID for next iteration: kp=%.6f ki=%.6f kd=%.6f (improving=%b)",
            currentKp, currentKi, currentKd, improving));
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
            ShooterConfig.kp, ShooterConfig.ki, ShooterConfig.kd, ShooterConfig.kf,  ShooterConfig.kfBase);

        // Initialize shooter2
        DcMotor motor2 = hardwareMap.dcMotor.get("shooter2");
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter2 = new KMotor(motor2,
            ShooterConfig.kp, ShooterConfig.ki, ShooterConfig.kd, ShooterConfig.kf,  ShooterConfig.kfBase);

        KLog.d("PIDIterativeLearning", "Motors initialized successfully");
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
     * Simple class to hold test results for one target RPS
     */
    private static class TestResult {
        double rampUpTime = 0.0;  // Time to reach target (milliseconds)
        double maxRPS = 0.0;      // Maximum RPS during stability window
        double minRPS = 0.0;      // Minimum RPS during stability window
    }
}
