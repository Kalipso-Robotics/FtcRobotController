package com.kalipsorobotics.test;

import com.kalipsorobotics.actions.turret.TurretConfig;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KMotor;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;

/**
 * TurretDataCollector - A test OpMode for collecting PIDF tuning data
 *
 * This class runs the turret to many different target angles and records:
 * - Time to reach within tolerance of target position
 * - Overshoot behavior during settling
 * - Real position vs target position over time
 *
 * Data is logged to logcat via KLog.d for analysis and PIDF coefficient tuning.
 *
 * IMPORTANT: The starting position when program runs is treated as 0 degrees.
 * The turret will only move ±180 degrees from that starting position.
 */
@TeleOp(name = "Turret Data Collector", group = "Test")
public class TurretDataCollector extends LinearOpMode {

    private static final String TAG = "TurretDataCollector";
    private static final String TAG_TRIAL = "TurretDataCollector_Trial";
    private static final String TAG_SUMMARY = "TurretDataCollector_Summary";
    private static final String TAG_POWER = "TurretDataCollector_Power";
    private static final String TAG_SETTLE = "TurretDataCollector_Settle";

    // Test configuration
    private static final double TOLERANCE_DEGREES = 1.0;
    private static final double TOLERANCE_TICKS = Turret.TICKS_PER_DEGREE * TOLERANCE_DEGREES;
    private static final long MAX_MOVE_TIME_MS = 3000; // Max time to reach target
    private static final long SETTLE_TIME_MS = 1000; // Time to observe settling behavior
    private static final long SETTLE_LOG_INTERVAL_MS = 50; // How often to log during settling

    // Turret limits (±180 degrees from starting position)
    private static final double MAX_ANGLE_DEG = 180.0;
    private static final double MIN_ANGLE_DEG = -180.0;
    private static final double MAX_TICKS_FROM_ZERO = MAX_ANGLE_DEG * Turret.TICKS_PER_DEGREE;
    private static final double MIN_TICKS_FROM_ZERO = MIN_ANGLE_DEG * Turret.TICKS_PER_DEGREE;

    // Test angles in degrees - relative to starting position, stays within ±180
    // ~55 trials, estimated 5-10 minutes
    private static final double[] TEST_ANGLES_DEG = {
        // Sweep negative (15 deg increments)
        0, -15, -30, -45, -60, -75, -90, -105, -120, -135, -150, -165, -180,
        // Sweep back to center
        -165, -150, -135, -120, -105, -90, -75, -60, -45, -30, -15, 0,
        // Sweep positive (15 deg increments)
        15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180,
        // Sweep back to center
        165, 150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 0,
        // Large alternating jumps
        -90, 90, -180, 180, -45, 45, -135, 135, 0
    };

    private Turret turret;
    private KMotor turretMotor;
    private OpModeUtilities opModeUtilities;

    // Zero offset - the encoder position when we started (this is our "0 degrees")
    private int zeroOffsetTicks = 0;

    // Velocity tracking (mirrored from TurretAutoAlignLimelight)
    private double previousAngleTicks;
    private double currentAngularVelocity; // rad per ms
    private ElapsedTime velocityTimer;
    private boolean isFirstVelocityUpdate;

    // Trial statistics
    private int totalTrials = 0;
    private int successfulTrials = 0;
    private double totalSettleTimeMs = 0;
    private double maxOvershootDeg = 0;
    private double minOvershootDeg = Double.MAX_VALUE;

    // Lists for percentile calculations
    private ArrayList<Long> settleTimesList = new ArrayList<>();
    private ArrayList<Double> overshootList = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        turret = Turret.getInstance(opModeUtilities);
        turretMotor = turret.getTurretMotor();

        // CRITICAL: Store the starting position as our zero reference
        zeroOffsetTicks = turretMotor.getCurrentPosition();

        // Initialize velocity tracking
        velocityTimer = new ElapsedTime();
        previousAngleTicks = 0;
        currentAngularVelocity = 0;
        isFirstVelocityUpdate = true;

        // Log configuration
        logConfiguration();

        telemetry.addLine("Turret Data Collector Ready");
        telemetry.addLine("Current position is now ZERO");
        telemetry.addData("Zero offset", zeroOffsetTicks + " ticks");
        telemetry.addData("Test angles", TEST_ANGLES_DEG.length);
        telemetry.addData("Tolerance", TOLERANCE_DEGREES + " deg");
        telemetry.addData("Limits", MIN_ANGLE_DEG + " to " + MAX_ANGLE_DEG + " deg");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            runDataCollection();
        }

        // Log final summary
        logFinalSummary();
    }

    /**
     * Get current position relative to where we started (in ticks)
     */
    private int getRelativePositionTicks() {
        return turretMotor.getCurrentPosition() - zeroOffsetTicks;
    }

    /**
     * Get current position relative to where we started (in degrees)
     */
    private double getRelativePositionDeg() {
        return getRelativePositionTicks() / Turret.TICKS_PER_DEGREE;
    }

    /**
     * Convert target degrees to absolute encoder ticks (accounting for zero offset)
     */
    private int targetDegreesToAbsoluteTicks(double targetDeg) {
        // Clamp target to ±180 degrees
        double clampedDeg = Math.max(MIN_ANGLE_DEG, Math.min(MAX_ANGLE_DEG, targetDeg));
        return zeroOffsetTicks + (int)(clampedDeg * Turret.TICKS_PER_DEGREE);
    }

    private void logConfiguration() {
        KLog.d(TAG, "=== TURRET DATA COLLECTOR CONFIGURATION ===");
        KLog.d(TAG, String.format("PIDF Coefficients: kP=%.6f, kI=%.6f, kD=%.6f, kF=%.3f, kS=%.3f",
                TurretConfig.kP, TurretConfig.kI, TurretConfig.kD, TurretConfig.kF, TurretConfig.kS));
        KLog.d(TAG, String.format("Turret Constants: TICKS_PER_DEGREE=%.4f, TICKS_PER_RADIAN=%.4f",
                Turret.TICKS_PER_DEGREE, Turret.TICKS_PER_RADIAN));
        KLog.d(TAG, String.format("Zero offset: %d ticks (starting position treated as 0 deg)", zeroOffsetTicks));
        KLog.d(TAG, String.format("Limits: %.1f to %.1f deg (%.1f to %.1f ticks from zero)",
                MIN_ANGLE_DEG, MAX_ANGLE_DEG, MIN_TICKS_FROM_ZERO, MAX_TICKS_FROM_ZERO));
        KLog.d(TAG, String.format("Test Config: Tolerance=%.2f deg (%.2f ticks), MaxMoveTime=%d ms, SettleTime=%d ms",
                TOLERANCE_DEGREES, TOLERANCE_TICKS, MAX_MOVE_TIME_MS, SETTLE_TIME_MS));
        KLog.d(TAG, "Test angles count: " + TEST_ANGLES_DEG.length);
        KLog.d(TAG, "============================================");
    }

    private void runDataCollection() {
        KLog.d(TAG, "=== STARTING DATA COLLECTION ===");
        KLog.d(TAG, String.format("Zero offset: %d ticks", zeroOffsetTicks));
        KLog.d(TAG, String.format("Initial relative position: %d ticks (%.2f deg)",
                getRelativePositionTicks(), getRelativePositionDeg()));

        for (int i = 0; i < TEST_ANGLES_DEG.length && opModeIsActive(); i++) {
            double targetAngleDeg = TEST_ANGLES_DEG[i];

            // Clamp target to ±180 degrees
            targetAngleDeg = Math.max(MIN_ANGLE_DEG, Math.min(MAX_ANGLE_DEG, targetAngleDeg));

            // Calculate absolute encoder target (accounting for zero offset)
            int targetAbsoluteTicks = targetDegreesToAbsoluteTicks(targetAngleDeg);

            // Get current position (relative to start)
            int currentRelativeTicks = getRelativePositionTicks();
            double currentRelativeDeg = getRelativePositionDeg();

            totalTrials++;
            KLog.d(TAG_TRIAL, "====================================================");
            KLog.d(TAG_TRIAL, String.format("TRIAL %d/%d", i + 1, TEST_ANGLES_DEG.length));
            KLog.d(TAG_TRIAL, String.format("Target: %.1f deg", targetAngleDeg));
            KLog.d(TAG_TRIAL, String.format("Target absolute ticks: %d", targetAbsoluteTicks));
            KLog.d(TAG_TRIAL, String.format("Current: %.2f deg (%d ticks relative)", currentRelativeDeg, currentRelativeTicks));
            KLog.d(TAG_TRIAL, String.format("Movement needed: %.2f deg", targetAngleDeg - currentRelativeDeg));

            // Run the trial
            TrialResult result = runSingleTrial(targetAbsoluteTicks, targetAngleDeg);

            // Only observe settling if we reached the target
            if (result.reachedTarget) {
                observeSettling(targetAbsoluteTicks, targetAngleDeg, result);
                successfulTrials++;
                totalSettleTimeMs += result.settleTimeMs;

                // Add to lists for percentile calculations
                settleTimesList.add(result.settleTimeMs);
                overshootList.add(result.maxOvershootDeg);

                // Track min/max overshoot
                if (result.maxOvershootDeg > maxOvershootDeg) {
                    maxOvershootDeg = result.maxOvershootDeg;
                }
                if (result.maxOvershootDeg < minOvershootDeg) {
                    minOvershootDeg = result.maxOvershootDeg;
                }
            } else {
                KLog.d(TAG_TRIAL, "Skipping settling observation - target was not reached");
            }

            logTrialResult(i + 1, targetAngleDeg, targetAbsoluteTicks, result);

            // Brief pause between trials
            sleep(200);

            // Update telemetry between trials
            telemetry.addData("Completed", "%d/%d trials", i + 1, TEST_ANGLES_DEG.length);
            telemetry.addData("Success Rate", "%.1f%%", (successfulTrials * 100.0 / totalTrials));
            telemetry.addData("Last Settle Time", "%d ms", result.settleTimeMs);
            telemetry.addData("Max Overshoot", "%.2f deg", maxOvershootDeg);
            telemetry.addData("Current Position", "%.1f deg", getRelativePositionDeg());
            telemetry.update();
        }

        // Stop turret at end
        turretMotor.stop();
        KLog.d(TAG, "=== DATA COLLECTION COMPLETE ===");
    }

    private TrialResult runSingleTrial(int targetAbsoluteTicks, double targetAngleDeg) {
        TrialResult result = new TrialResult();
        ElapsedTime trialTimer = new ElapsedTime();

        // Reset velocity tracking for new trial
        isFirstVelocityUpdate = true;
        turretMotor.getPIDFController().reset();

        int initialRelativeTicks = getRelativePositionTicks();
        double initialRelativeDeg = getRelativePositionDeg();
        boolean hasReachedTarget = false;

        KLog.d(TAG_TRIAL, String.format("Starting from: %.2f deg (%d ticks relative)",
                initialRelativeDeg, initialRelativeTicks));

        while (trialTimer.milliseconds() < MAX_MOVE_TIME_MS && opModeIsActive()) {
            // Update angular velocity
            updateAngularVelocity();

            int currentAbsoluteTicks = turretMotor.getCurrentPosition();
            int currentRelativeTicks = currentAbsoluteTicks - zeroOffsetTicks;
            double currentRelativeDeg = currentRelativeTicks / Turret.TICKS_PER_DEGREE;

            int error = targetAbsoluteTicks - currentAbsoluteTicks;
            double errorDeg = error / Turret.TICKS_PER_DEGREE;

            // Safety check - only stop if we're outside limits AND trying to go further out
            boolean outsidePositiveLimit = currentRelativeTicks > MAX_TICKS_FROM_ZERO + TOLERANCE_TICKS;
            boolean outsideNegativeLimit = currentRelativeTicks < MIN_TICKS_FROM_ZERO - TOLERANCE_TICKS;
            boolean targetIsAwayFromCenter = (outsidePositiveLimit && targetAbsoluteTicks > currentAbsoluteTicks) ||
                                              (outsideNegativeLimit && targetAbsoluteTicks < currentAbsoluteTicks);

            if ((outsidePositiveLimit || outsideNegativeLimit) && targetIsAwayFromCenter) {
                KLog.d(TAG_TRIAL, String.format("SAFETY STOP - Position %.2f deg exceeds limits and target is further out!", currentRelativeDeg));
                turretMotor.stop();
                result.reachedTarget = false;
                break;
            }

            // Check if within range
            boolean isWithinRange = Math.abs(error) < Math.abs(TOLERANCE_TICKS);

            if (isWithinRange) {
                if (!hasReachedTarget) {
                    hasReachedTarget = true;
                    result.settleTimeMs = (long) trialTimer.milliseconds();
                    result.reachedTarget = true;
                    KLog.d(TAG_TRIAL, String.format("REACHED TARGET at %d ms", result.settleTimeMs));
                    KLog.d(TAG_TRIAL, String.format("  Position: %.2f deg (%d ticks relative)",
                            currentRelativeDeg, currentRelativeTicks));
                    KLog.d(TAG_TRIAL, String.format("  Error: %.3f deg", errorDeg));
                }
                // Stop motor when within range
                turretMotor.stop();
                // Exit the loop - we're done with movement phase
                break;
            } else {
                // Calculate power using PID only (no feedforward for static targets)
                // Feedforward in TurretAutoAlignLimelight is for tracking a MOVING target angle
                // Here our target is static, so feedforward = 0
                double pidOutput = turretMotor.getPIDFController().calculate(error);

                double totalPower = Math.max(-1.0, Math.min(1.0, pidOutput));

                KLog.d(TAG_POWER, String.format("t=%4.0f | Pos: %6.1f deg | Err: %6.2f deg | PID: %6.4f | Pwr: %6.4f",
                        trialTimer.milliseconds(), currentRelativeDeg, errorDeg, pidOutput, totalPower));

                turretMotor.setPower(totalPower);

                // Track overshoot (if we passed the target)
                if (hasReachedTarget) {
                    double overshoot = Math.abs(errorDeg);
                    if (overshoot > result.maxOvershootDeg) {
                        result.maxOvershootDeg = overshoot;
                        KLog.d(TAG_TRIAL, String.format("OVERSHOOT: %.2f deg past target", overshoot));
                    }
                }
            }

            // Update telemetry
            telemetry.addData("Trial", "%d/%d", totalTrials, TEST_ANGLES_DEG.length);
            telemetry.addData("Target", "%.1f deg", targetAngleDeg);
            telemetry.addData("Current", "%.2f deg", currentRelativeDeg);
            telemetry.addData("Error", "%.2f deg", errorDeg);
            telemetry.addData("Time", "%.0f ms", trialTimer.milliseconds());
            telemetry.addData("Within Range", isWithinRange);
            telemetry.update();

            sleep(10); // Control loop rate
        }

        result.totalTimeMs = (long) trialTimer.milliseconds();
        result.finalPositionDeg = getRelativePositionDeg();
        result.finalPositionTicks = getRelativePositionTicks();
        result.finalErrorDeg = targetAngleDeg - result.finalPositionDeg;

        if (!result.reachedTarget) {
            turretMotor.stop();
            KLog.d(TAG_TRIAL, String.format("TIMEOUT after %d ms - Target not reached", MAX_MOVE_TIME_MS));
            KLog.d(TAG_TRIAL, String.format("  Final position: %.2f deg (%d ticks relative)",
                    result.finalPositionDeg, (int) result.finalPositionTicks));
            KLog.d(TAG_TRIAL, String.format("  Final error: %.2f deg", result.finalErrorDeg));
        }

        return result;
    }

    /**
     * Observe the turret position during settling to detect oscillation and overshoot
     */
    private void observeSettling(int targetAbsoluteTicks, double targetAngleDeg, TrialResult result) {
        ElapsedTime settleTimer = new ElapsedTime();
        ElapsedTime logTimer = new ElapsedTime();

        double minPositionDeg = Double.MAX_VALUE;
        double maxPositionDeg = Double.MIN_VALUE;
        int oscillationCount = 0;
        boolean wasPositive = false;
        boolean firstCheck = true;

        KLog.d(TAG_SETTLE, String.format("--- SETTLING OBSERVATION for %.1f deg target ---", targetAngleDeg));

        while (settleTimer.milliseconds() < SETTLE_TIME_MS && opModeIsActive()) {
            updateAngularVelocity();

            int currentAbsoluteTicks = turretMotor.getCurrentPosition();
            double currentRelativeDeg = (currentAbsoluteTicks - zeroOffsetTicks) / Turret.TICKS_PER_DEGREE;
            int error = targetAbsoluteTicks - currentAbsoluteTicks;
            double errorDeg = error / Turret.TICKS_PER_DEGREE;

            // Track min/max position
            if (currentRelativeDeg < minPositionDeg) minPositionDeg = currentRelativeDeg;
            if (currentRelativeDeg > maxPositionDeg) maxPositionDeg = currentRelativeDeg;

            // Detect oscillations (error sign changes)
            boolean isPositive = error > 0;
            if (!firstCheck && isPositive != wasPositive) {
                oscillationCount++;
            }
            wasPositive = isPositive;
            firstCheck = false;

            // Maintain position control during settling (PID only, no feedforward)
            if (Math.abs(error) >= TOLERANCE_TICKS) {
                double pidOutput = turretMotor.getPIDFController().calculate(error);
                double totalPower = Math.max(-1.0, Math.min(1.0, pidOutput));
                turretMotor.setPower(totalPower);

                // Track overshoot
                double overshoot = Math.abs(errorDeg);
                if (overshoot > result.maxOvershootDeg) {
                    result.maxOvershootDeg = overshoot;
                }
            } else {
                turretMotor.stop();
            }

            // Log at intervals
            if (logTimer.milliseconds() >= SETTLE_LOG_INTERVAL_MS) {
                KLog.d(TAG_SETTLE, String.format("t=%4.0f ms | Pos: %7.2f deg | Err: %6.2f deg | AngVel: %.6f rad/ms",
                        settleTimer.milliseconds(), currentRelativeDeg, errorDeg, currentAngularVelocity));
                logTimer.reset();
            }

            sleep(5);
        }

        // Final position after settling
        double finalPositionDeg = getRelativePositionDeg();
        double finalErrorDeg = targetAngleDeg - finalPositionDeg;
        double positionRangeDeg = maxPositionDeg - minPositionDeg;

        result.oscillationCount = oscillationCount;
        result.positionRangeDeg = positionRangeDeg;
        result.finalPositionDeg = finalPositionDeg;
        result.finalPositionTicks = getRelativePositionTicks();
        result.finalErrorDeg = finalErrorDeg;

        KLog.d(TAG_SETTLE, "--- SETTLING COMPLETE ---");
        KLog.d(TAG_SETTLE, String.format("Final position: %.2f deg (%d ticks relative)",
                finalPositionDeg, getRelativePositionTicks()));
        KLog.d(TAG_SETTLE, String.format("Final error: %.3f deg", finalErrorDeg));
        KLog.d(TAG_SETTLE, String.format("Position range during settle: %.3f deg (min: %.2f, max: %.2f)",
                positionRangeDeg, minPositionDeg, maxPositionDeg));
        KLog.d(TAG_SETTLE, String.format("Oscillation count: %d", oscillationCount));
        KLog.d(TAG_SETTLE, String.format("Max overshoot: %.3f deg", result.maxOvershootDeg));
    }

    /**
     * Update angular velocity tracking
     */
    private void updateAngularVelocity() {
        double currentTicks = turretMotor.getCurrentPosition();

        if (isFirstVelocityUpdate) {
            previousAngleTicks = currentTicks;
            currentAngularVelocity = 0;
            velocityTimer.reset();
            isFirstVelocityUpdate = false;
        } else {
            double deltaTime = velocityTimer.milliseconds();
            if (deltaTime > 0) {
                double deltaTicks = currentTicks - previousAngleTicks;
                double deltaRadians = deltaTicks / Turret.TICKS_PER_RADIAN;
                currentAngularVelocity = deltaRadians / deltaTime;

                previousAngleTicks = currentTicks;
                velocityTimer.reset();
            }
        }
    }

    private void logTrialResult(int trialNum, double targetAngleDeg, int targetAbsoluteTicks, TrialResult result) {
        KLog.d(TAG_SUMMARY, "====================================================");
        KLog.d(TAG_SUMMARY, String.format("TRIAL %d COMPLETE", trialNum));
        KLog.d(TAG_SUMMARY, String.format("  Target: %.1f deg (absolute ticks: %d)", targetAngleDeg, targetAbsoluteTicks));
        KLog.d(TAG_SUMMARY, String.format("  Success: %b", result.reachedTarget));
        KLog.d(TAG_SUMMARY, String.format("  Time to reach: %d ms", result.settleTimeMs));
        KLog.d(TAG_SUMMARY, String.format("  Final position: %.2f deg (%d ticks relative)",
                result.finalPositionDeg, (int) result.finalPositionTicks));
        KLog.d(TAG_SUMMARY, String.format("  Final error: %.3f deg", result.finalErrorDeg));
        KLog.d(TAG_SUMMARY, String.format("  Max overshoot: %.3f deg", result.maxOvershootDeg));
        KLog.d(TAG_SUMMARY, String.format("  Oscillations: %d", result.oscillationCount));
        KLog.d(TAG_SUMMARY, String.format("  Position range: %.3f deg", result.positionRangeDeg));
        KLog.d(TAG_SUMMARY, String.format("  PIDF: kP=%.6f, kI=%.6f, kD=%.6f, kF=%.3f, kS=%.3f",
                TurretConfig.kP, TurretConfig.kI, TurretConfig.kD, TurretConfig.kF, TurretConfig.kS));
    }

    private void logFinalSummary() {
        KLog.d(TAG, "====================================================");
        KLog.d(TAG, "============ FINAL SUMMARY ============");
        KLog.d(TAG, String.format("Total Trials: %d", totalTrials));
        KLog.d(TAG, String.format("Successful Trials: %d (%.1f%%)", successfulTrials,
                (successfulTrials * 100.0 / Math.max(1, totalTrials))));

        // Calculate percentiles if we have data
        long p90SettleTime = 0;
        double p90Overshoot = 0;

        if (successfulTrials > 0) {
            double avgTime = totalSettleTimeMs / successfulTrials;
            KLog.d(TAG, String.format("Average Time to Target: %.1f ms", avgTime));

            // Calculate P90 for settle time
            Collections.sort(settleTimesList);
            int p90Index = (int) Math.ceil(settleTimesList.size() * 0.90) - 1;
            p90Index = Math.max(0, Math.min(p90Index, settleTimesList.size() - 1));
            p90SettleTime = settleTimesList.get(p90Index);
            KLog.d(TAG, String.format("P90 Time to Target: %d ms", p90SettleTime));

            // Calculate P90 for overshoot
            Collections.sort(overshootList);
            p90Index = (int) Math.ceil(overshootList.size() * 0.90) - 1;
            p90Index = Math.max(0, Math.min(p90Index, overshootList.size() - 1));
            p90Overshoot = overshootList.get(p90Index);
            KLog.d(TAG, String.format("P90 Overshoot: %.3f deg", p90Overshoot));

            // Fix minOvershootDeg if no trials succeeded
            if (minOvershootDeg == Double.MAX_VALUE) {
                minOvershootDeg = 0;
            }
        }

        KLog.d(TAG, String.format("Min Overshoot: %.3f deg", minOvershootDeg == Double.MAX_VALUE ? 0 : minOvershootDeg));
        KLog.d(TAG, String.format("Max Overshoot: %.3f deg", maxOvershootDeg));
        KLog.d(TAG, String.format("PIDF Used: kP=%.6f, kI=%.6f, kD=%.6f, kF=%.3f, kS=%.3f",
                TurretConfig.kP, TurretConfig.kI, TurretConfig.kD, TurretConfig.kF, TurretConfig.kS));
        KLog.d(TAG, "========================================");

        telemetry.addLine("=== DATA COLLECTION COMPLETE ===");
        telemetry.addData("Total Trials", totalTrials);
        telemetry.addData("Successful", successfulTrials);
        telemetry.addData("Success Rate", "%.1f%%", (successfulTrials * 100.0 / Math.max(1, totalTrials)));
        if (successfulTrials > 0) {
            telemetry.addData("Avg Time to Target", "%.1f ms", totalSettleTimeMs / successfulTrials);
            telemetry.addData("P90 Time to Target", "%d ms", p90SettleTime);
            telemetry.addData("P90 Overshoot", "%.3f deg", p90Overshoot);
        }
        telemetry.addData("Min Overshoot", "%.3f deg", minOvershootDeg == Double.MAX_VALUE ? 0 : minOvershootDeg);
        telemetry.addData("Max Overshoot", "%.3f deg", maxOvershootDeg);
        telemetry.addLine("Check logcat for detailed data");
        telemetry.update();
    }

    /**
     * Container for trial results
     */
    private static class TrialResult {
        boolean reachedTarget = false;
        long settleTimeMs = 0;
        long totalTimeMs = 0;
        double finalPositionTicks = 0;
        double finalPositionDeg = 0;
        double finalErrorDeg = 0;
        double maxOvershootDeg = 0;
        int oscillationCount = 0;
        double positionRangeDeg = 0;
    }
}
