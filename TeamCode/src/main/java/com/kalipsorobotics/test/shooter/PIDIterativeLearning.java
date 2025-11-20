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
    private static final int MAX_ITERATIONS = 30;  // Number of learning iterations
    private static final double GRADIENT_STEP = 0.05;  // 5% perturbation for gradient estimation
    private static final double LEARNING_RATE = 0.3;  // Step size multiplier for updates
    private static final double MOMENTUM = 0.7;  // Momentum factor (0.0-0.9)

    // Target RPS test range
    private static final double RPS_START = 15.0;
    private static final double RPS_END = 60.0;
    private static final double RPS_STEP = 5.0;  // Test every 5 RPS

    // Test parameters
    private static final double RPS_TOLERANCE = 0.5;  // Consider "at target" when within ±0.5 RPS
    private static final long MAX_RAMP_TIME_MS = 8000;  // Max time to wait for ramp-up (8 seconds)
    private static final long STABILITY_WINDOW_MS = 500;  // Monitor stability for 500ms after reaching target
    private static final long SAMPLE_INTERVAL_MS = 50;  // Sample RPS every 50ms
    private static final int STALL_CHECK_SAMPLES = 10;  // Number of samples to check for stall
    private static final double STALL_THRESHOLD = 0.3;  // RPS change threshold to detect stall

    // Cost function weights (adjust to prioritize different factors)
    // Higher weight = more important to minimize
    private static final double WEIGHT_RAMP_TIME = 1.0;  // Fast ramp-up
    private static final double WEIGHT_OVERSHOOT = 2.0;  // Avoid overshoot (most important)
    private static final double WEIGHT_UNDERSHOOT = 1.5;  // Avoid undershoot
    private static final double WEIGHT_INSTABILITY = 1.0;  // Minimize oscillation
    private static final double TIMEOUT_PENALTY = 1000.0;  // Large penalty for failing to reach target

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

    // Gradient descent with momentum
    private double velocityKp = 0.0;
    private double velocityKi = 0.0;
    private double velocityKd = 0.0;

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
        fileWriter.writeLine("Iteration,targetRPS,kp,ki,kd,rampUpTime,maxRPS,minRPS,reachedTarget,stalled");

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
        // Each iteration: 1 focus test + 3 gradient tests = 4 tests
        // Plus final validation across all RPS
        int validationTests = (int) ((RPS_END - RPS_START) / RPS_STEP) + 1;
        int totalTests = (MAX_ITERATIONS * 4) + validationTests;
        int estimatedTimeMinutes = (totalTests * 9) / 60;  // ~9 seconds per test

        telemetry.addLine("Initialization complete!");
        telemetry.addLine();
        telemetry.addLine("Learning Configuration:");
        telemetry.addData("Algorithm", "Gradient Descent w/ Momentum");
        telemetry.addData("Max Iterations", MAX_ITERATIONS);
        telemetry.addData("Focus RPS", "40.0");
        telemetry.addData("Total Tests", totalTests);
        telemetry.addData("Est. Time", "%d min", estimatedTimeMinutes);
        telemetry.addLine();
        telemetry.addLine("Initial PID Values:");
        telemetry.addData("Kp", "%.6f", currentKp);
        telemetry.addData("Ki", "%.6f", currentKi);
        telemetry.addData("Kd", "%.6f", currentKd);
        telemetry.addData("Kf (fixed)", "%.6f", currentKf);
        telemetry.addLine();
        telemetry.addLine("Improvements:");
        telemetry.addLine("- Gradient-based search (not random)");
        telemetry.addLine("- Early stall detection");
        telemetry.addLine("- Fixed cost normalization");
        telemetry.addLine("- Stability only after target reached");
        telemetry.addLine();
        telemetry.addLine("Press PLAY to start learning");
        telemetry.update();

        KLog.d("PIDIterativeLearning", String.format("Starting with PID: kp=%.6f, ki=%.6f, kd=%.6f, kf=%.6f",
            currentKp, currentKi, currentKd, currentKf));

        waitForStart();

        // ========== MAIN LEARNING LOOP ==========
        // Focus on one critical RPS value at a time for better convergence
        // Start with a mid-range value (40 RPS) which is commonly used
        double focusRPS = 40.0;

        for (int iteration = 1; iteration <= MAX_ITERATIONS && opModeIsActive(); iteration++) {

            telemetry.addLine("=== PID Iterative Learning ===");
            telemetry.addData("Iteration", "%d / %d", iteration, MAX_ITERATIONS);
            telemetry.addData("Focus RPS", "%.1f", focusRPS);
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

            KLog.d("PIDIterativeLearning", String.format("========== Iteration %d/%d (Focus: %.1f RPS) ==========",
                iteration, MAX_ITERATIONS, focusRPS));
            KLog.d("PIDIterativeLearning", String.format("Testing PID: kp=%.6f, ki=%.6f, kd=%.6f",
                currentKp, currentKi, currentKd));

            // Apply current PID values to motors
            shooter1.getPIDFController().setKp(currentKp);
            shooter1.getPIDFController().setKi(currentKi);
            shooter1.getPIDFController().setKd(currentKd);

            shooter2.getPIDFController().setKp(currentKp);
            shooter2.getPIDFController().setKi(currentKi);
            shooter2.getPIDFController().setKd(currentKd);

            // Run test at focus RPS
            telemetry.addData("Testing RPS", "%.1f", focusRPS);
            telemetry.update();

            TestResult result = runSingleTest(focusRPS, iteration, 1);

            // Calculate cost for this test
            double cost = calculateCost(result, focusRPS);

            // Log results to file
            String line = String.format("%d,%.1f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%b,%b",
                iteration, focusRPS, currentKp, currentKi, currentKd,
                result.rampUpTime, result.maxRPS, result.minRPS, result.reachedTarget, result.stalled);
            fileWriter.writeLine(line);

            KLog.d("PIDIterativeLearning", String.format(
                "Iter %d | RPS %.1f | Ramp %.0fms | Max %.2f | Min %.2f | Reached: %b | Cost %.2f",
                iteration, focusRPS, result.rampUpTime, result.maxRPS, result.minRPS, result.reachedTarget, cost));

            telemetry.addData("Test Cost", "%.2f", cost);
            telemetry.addData("Reached Target", result.reachedTarget ? "YES" : "NO");
            telemetry.update();

            // Check if this is the best performance so far
            if (cost < bestCost) {
                bestCost = cost;
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

            // Update PID values for next iteration using gradient descent
            if (iteration < MAX_ITERATIONS) {
                updatePIDValues(cost, focusRPS);
            }

            sleep(300);  // Brief pause between iterations
        }

        // ========== VALIDATION ACROSS ALL RPS ==========
        // Test the best PID values across full range
        telemetry.addLine("=== Validation Phase ===");
        telemetry.addLine("Testing best PID across all RPS...");
        telemetry.update();

        KLog.d("PIDIterativeLearning", "========== Validation Phase ==========");
        KLog.d("PIDIterativeLearning", String.format("Testing best PID: kp=%.6f, ki=%.6f, kd=%.6f",
            bestKp, bestKi, bestKd));

        // Apply best values
        shooter1.getPIDFController().setKp(bestKp);
        shooter1.getPIDFController().setKi(bestKi);
        shooter1.getPIDFController().setKd(bestKd);
        shooter2.getPIDFController().setKp(bestKp);
        shooter2.getPIDFController().setKi(bestKi);
        shooter2.getPIDFController().setKd(bestKd);

        for (double targetRPS = RPS_START; targetRPS <= RPS_END + 0.01 && opModeIsActive(); targetRPS += RPS_STEP) {
            telemetry.addData("Validating RPS", "%.1f", targetRPS);
            telemetry.update();

            TestResult result = runSingleTest(targetRPS, 999, 0);
            double cost = calculateCost(result, targetRPS);

            String line = String.format("VALIDATION,%.1f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%b,%b",
                targetRPS, bestKp, bestKi, bestKd,
                result.rampUpTime, result.maxRPS, result.minRPS, result.reachedTarget, result.stalled);
            fileWriter.writeLine(line);

            KLog.d("PIDIterativeLearning", String.format(
                "Validation | RPS %.1f | Ramp %.0fms | Max %.2f | Min %.2f | Reached: %b | Cost %.2f",
                targetRPS, result.rampUpTime, result.maxRPS, result.minRPS, result.reachedTarget, cost));
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
        ArrayList<Double> recentRPS = new ArrayList<>();

        while (opModeIsActive() && timer.milliseconds() < MAX_RAMP_TIME_MS) {
            // Continuously call goToRPS to update PID control
            shooter1.goToRPS(targetRPS);
            shooter2.goToRPS(targetRPS);

            double currentRPS = shooter1.getRPS();

            // Track recent RPS for stall detection
            recentRPS.add(currentRPS);
            if (recentRPS.size() > STALL_CHECK_SAMPLES) {
                recentRPS.remove(0);
            }

            // Check if we've reached target (within ±0.5 RPS)
            if (Math.abs(currentRPS - targetRPS) <= RPS_TOLERANCE) {
                result.reachedTarget = true;
                result.rampUpTime = timer.milliseconds();
                KLog.d("PIDIterativeLearning", String.format(
                    "Reached target %.1f RPS in %.0fms (actual: %.2f)",
                    targetRPS, result.rampUpTime, currentRPS));
                break;
            }

            // Check for stall (RPS not changing significantly)
            if (recentRPS.size() >= STALL_CHECK_SAMPLES && timer.milliseconds() > 2000) {
                double minRecent = calculateMin(recentRPS);
                double maxRecent = calculateMax(recentRPS);
                double rpsRange = maxRecent - minRecent;

                if (rpsRange < STALL_THRESHOLD) {
                    result.stalled = true;
                    result.rampUpTime = timer.milliseconds();
                    KLog.d("PIDIterativeLearning", String.format(
                        "Stalled: RPS stuck at %.2f (target %.1f) - range %.2f over %d samples",
                        currentRPS, targetRPS, rpsRange, STALL_CHECK_SAMPLES));
                    break;
                }
            }

            sleep(SAMPLE_INTERVAL_MS);
        }

        // If didn't reach target within timeout, record the timeout
        if (!result.reachedTarget) {
            result.rampUpTime = timer.milliseconds();
            if (!result.stalled) {
                KLog.d("PIDIterativeLearning", String.format(
                    "Timeout: Did not reach target RPS %.1f within %dms",
                    targetRPS, MAX_RAMP_TIME_MS));
            }
        }

        // ========== STABILITY MONITORING PHASE ==========
        // Collect RPS data for 500ms after reaching target
        // This measures overshoot, undershoot, and oscillation
        // ONLY run if target was successfully reached
        if (result.reachedTarget) {
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
                result.maxRPS = targetRPS;
                result.minRPS = targetRPS;
            }
        } else {
            // Target not reached - set values to current RPS for undershoot calculation
            double finalRPS = shooter1.getRPS();
            result.maxRPS = finalRPS;
            result.minRPS = finalRPS;
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
     * 1. Ramp-up time (normalized by target RPS ONLY if successful)
     * 2. Overshoot (how much max RPS exceeds target - bad for accuracy)
     * 3. Undershoot (how much min RPS falls below target - indicates instability)
     * 4. Instability (range of oscillation during stability window)
     * 5. Timeout penalty (if target was never reached)
     *
     * @param result Test result data
     * @param targetRPS Target RPS for this test
     * @return Cost value (lower is better)
     */
    private double calculateCost(TestResult result, double targetRPS) {
        // If target was never reached, apply large penalty
        if (!result.reachedTarget) {
            // Large base penalty + undershoot penalty
            double undershoot = Math.max(0, targetRPS - result.maxRPS);
            return TIMEOUT_PENALTY + (WEIGHT_UNDERSHOOT * undershoot);
        }

        // Target was reached - calculate normal cost
        // Normalize ramp-up time by target RPS (higher RPS takes longer)
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
     * Update PID values for next iteration using gradient descent with momentum
     *
     * STRATEGY:
     * Uses finite differences to estimate gradient and momentum for smooth optimization:
     *
     * 1. Estimate gradient for each parameter by testing small perturbations
     * 2. Compute velocity update: velocity = momentum * old_velocity - learning_rate * gradient
     * 3. Update parameters: param = param + velocity
     * 4. Clamp to safe ranges
     *
     * This approach:
     * - Has directional memory (momentum prevents oscillation)
     * - Systematically moves in directions that improve cost
     * - Avoids random walk behavior
     * - Converges faster than random search
     *
     * @param currentCost Cost of current iteration
     * @param targetRPS The RPS value being optimized
     */
    private void updatePIDValues(double currentCost, double targetRPS) throws InterruptedException {
        KLog.d("PIDIterativeLearning", String.format(
            "Estimating gradients at: kp=%.6f ki=%.6f kd=%.6f (cost=%.2f)",
            currentKp, currentKi, currentKd, currentCost));

        // ========== ESTIMATE GRADIENT FOR Kp ==========
        double originalKp = currentKp;
        double gradKp;

        // Test positive perturbation - use additive step to avoid division by zero
        double deltaKp = Math.max(currentKp * GRADIENT_STEP, 0.000001);  // At least 1e-6
        double testKp = currentKp + deltaKp;
        testKp = Math.max(KP_MIN, Math.min(KP_MAX, testKp));

        // If clamping made no change, skip this gradient
        if (Math.abs(testKp - currentKp) < 1e-10) {
            gradKp = 0.0;
            KLog.d("PIDIterativeLearning", "Kp at boundary, gradient = 0");
        } else {
            shooter1.getPIDFController().setKp(testKp);
            shooter2.getPIDFController().setKp(testKp);
            TestResult kpTestResult = runSingleTest(targetRPS, -1, -1);
            double kpCost = calculateCost(kpTestResult, targetRPS);

            // Estimate gradient: (cost_new - cost_old) / delta_kp
            gradKp = (kpCost - currentCost) / (testKp - currentKp);

            KLog.d("PIDIterativeLearning", String.format(
                "Gradient Kp: %.6f (test cost: %.2f)", gradKp, kpCost));
        }

        // Restore original value
        shooter1.getPIDFController().setKp(originalKp);
        shooter2.getPIDFController().setKp(originalKp);

        // ========== ESTIMATE GRADIENT FOR Ki ==========
        double originalKi = currentKi;
        double gradKi;

        // Test positive perturbation - use additive step to avoid division by zero
        double deltaKi = Math.max(currentKi * GRADIENT_STEP, 0.0000001);  // At least 1e-7
        double testKi = currentKi + deltaKi;
        testKi = Math.max(KI_MIN, Math.min(KI_MAX, testKi));

        // If clamping made no change, skip this gradient
        if (Math.abs(testKi - currentKi) < 1e-10) {
            gradKi = 0.0;
            KLog.d("PIDIterativeLearning", "Ki at boundary, gradient = 0");
        } else {
            shooter1.getPIDFController().setKi(testKi);
            shooter2.getPIDFController().setKi(testKi);
            TestResult kiTestResult = runSingleTest(targetRPS, -1, -1);
            double kiCost = calculateCost(kiTestResult, targetRPS);

            // Estimate gradient
            gradKi = (kiCost - currentCost) / (testKi - currentKi);

            KLog.d("PIDIterativeLearning", String.format(
                "Gradient Ki: %.6f (test cost: %.2f)", gradKi, kiCost));
        }

        // Restore original value
        shooter1.getPIDFController().setKi(originalKi);
        shooter2.getPIDFController().setKi(originalKi);

        // ========== ESTIMATE GRADIENT FOR Kd ==========
        double originalKd = currentKd;
        double gradKd;

        // Test positive perturbation - use additive step to avoid division by zero
        double deltaKd = Math.max(currentKd * GRADIENT_STEP, 0.0000001);  // At least 1e-7
        double testKd = currentKd + deltaKd;
        testKd = Math.max(KD_MIN, Math.min(KD_MAX, testKd));

        // If clamping made no change, skip this gradient
        if (Math.abs(testKd - currentKd) < 1e-10) {
            gradKd = 0.0;
            KLog.d("PIDIterativeLearning", "Kd at boundary, gradient = 0");
        } else {
            shooter1.getPIDFController().setKd(testKd);
            shooter2.getPIDFController().setKd(testKd);
            TestResult kdTestResult = runSingleTest(targetRPS, -1, -1);
            double kdCost = calculateCost(kdTestResult, targetRPS);

            // Estimate gradient
            gradKd = (kdCost - currentCost) / (testKd - currentKd);

            KLog.d("PIDIterativeLearning", String.format(
                "Gradient Kd: %.6f (test cost: %.2f)", gradKd, kdCost));
        }

        // Restore original value
        shooter1.getPIDFController().setKd(originalKd);
        shooter2.getPIDFController().setKd(originalKd);

        // ========== UPDATE WITH MOMENTUM ==========
        // Velocity = momentum * old_velocity - learning_rate * gradient
        velocityKp = MOMENTUM * velocityKp - LEARNING_RATE * currentKp * gradKp;
        velocityKi = MOMENTUM * velocityKi - LEARNING_RATE * currentKi * gradKi;
        velocityKd = MOMENTUM * velocityKd - LEARNING_RATE * currentKd * gradKd;

        // Check for NaN in velocities and reset if necessary
        if (Double.isNaN(velocityKp) || Double.isInfinite(velocityKp)) {
            velocityKp = 0.0;
            KLog.d("PIDIterativeLearning", "WARNING: velocityKp was NaN/Inf, reset to 0");
        }
        if (Double.isNaN(velocityKi) || Double.isInfinite(velocityKi)) {
            velocityKi = 0.0;
            KLog.d("PIDIterativeLearning", "WARNING: velocityKi was NaN/Inf, reset to 0");
        }
        if (Double.isNaN(velocityKd) || Double.isInfinite(velocityKd)) {
            velocityKd = 0.0;
            KLog.d("PIDIterativeLearning", "WARNING: velocityKd was NaN/Inf, reset to 0");
        }

        // Update parameters
        currentKp += velocityKp;
        currentKi += velocityKi;
        currentKd += velocityKd;

        // Check for NaN in updated values and restore originals if necessary
        if (Double.isNaN(currentKp) || Double.isInfinite(currentKp)) {
            currentKp = originalKp;
            KLog.d("PIDIterativeLearning", "WARNING: currentKp became NaN/Inf, restored original");
        }
        if (Double.isNaN(currentKi) || Double.isInfinite(currentKi)) {
            currentKi = originalKi;
            KLog.d("PIDIterativeLearning", "WARNING: currentKi became NaN/Inf, restored original");
        }
        if (Double.isNaN(currentKd) || Double.isInfinite(currentKd)) {
            currentKd = originalKd;
            KLog.d("PIDIterativeLearning", "WARNING: currentKd became NaN/Inf, restored original");
        }

        // Clamp to safe ranges
        currentKp = Math.max(KP_MIN, Math.min(KP_MAX, currentKp));
        currentKi = Math.max(KI_MIN, Math.min(KI_MAX, currentKi));
        currentKd = Math.max(KD_MIN, Math.min(KD_MAX, currentKd));

        KLog.d("PIDIterativeLearning", String.format(
            "Updated PID: kp=%.6f ki=%.6f kd=%.6f (velocities: %.6f, %.6f, %.6f)",
            currentKp, currentKi, currentKd, velocityKp, velocityKi, velocityKd));
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
        boolean reachedTarget = false;  // True if target was reached successfully
        boolean stalled = false;  // True if ramp-up stalled before reaching target
    }
}
