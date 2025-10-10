package com.kalipsorobotics.actions.shooter;

import android.util.Log;
import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterReady extends Action {


    private final Shooter shooter;
    private final Point target;

    // PID constants - tune these values
    private static final double KP = 0.0007;  // Proportional gain
    private static final double KI = 0.0001;  // Integral gain
    private static final double KD = 0.0005;  // Derivative gain

    private static final double TARGET_RPS_TOLERANCE = 1.0;
    private static final double RPS_READ_DURATION_MS = 100; // Duration to read RPS
    private static final double MOTOR_ACCELERATION_WAIT_MS = 150; // Wait for motor to accelerate
    private static final double MAX_POWER = 1.0;
    private static final double MIN_POWER = 0.0;

    // State tracking
    private enum State {
        READING_RPS,
        WAITING_FOR_ACCELERATION
    }

    private State currentState = State.READING_RPS;
    private ElapsedTime stateTimer = null;

    // PID state
    private double previousError = 0;
    private double integral = 0;
    private double currentPower = MIN_POWER;
    private double previousTargetRPS = 0;

    // RPS accumulation
    private double rpsSum = 0;
    private int rpsReadCount = 0;

    public ShooterReady(Shooter shooter, Point target) {
        this.shooter = shooter;
        this.target = target;
        this.name = "ShooterReady";
    }

    @Override
    public void update() {
        if (!hasStarted) {
            hasStarted = true;
            previousError = 0;
            integral = 0;
            currentPower = MIN_POWER;
            previousTargetRPS = 0;
            currentState = State.READING_RPS;
            stateTimer = new ElapsedTime();
            rpsSum = 0;
            rpsReadCount = 0;
        }
        KLog.d("ShooterReadyPressed", "Shooter Ready Update");

        // Always update hood position based on current position
        shooter.updateHoodFromPosition(SharedData.getOdometryPosition(), target);

        switch (currentState) {
            case READING_RPS:
                handleReadingRPS();
                break;

            case WAITING_FOR_ACCELERATION:
                handleWaitingForAcceleration();
                break;
        }
    }

    private void handleReadingRPS() {
        // Read Current RPS for duration 100ms
        accumulateRPSReading();

        // Check if we've read for the full duration
        if (stateTimer.milliseconds() >= RPS_READ_DURATION_MS) {
            processRPSReadings();
        }
    }

    private void accumulateRPSReading() {
        double currentRPS = shooter.getRPS();
        rpsSum += currentRPS;
        rpsReadCount++;
    }

    private void processRPSReadings() {
        double averageRPS = calculateAverageRPS();
        double targetRPS = getTargetRPS();
        double error = targetRPS - averageRPS;

        // Reset PID if target changed significantly (>5 RPS)
        if (Math.abs(targetRPS - previousTargetRPS) > 5.0) {
            integral = 0;
            previousError = 0;
            Log.w("ShooterReady_PID", "Target changed from " + previousTargetRPS + " to " + targetRPS + " - Resetting PID");
        }
        previousTargetRPS = targetRPS;

        logRPSReadings(averageRPS, targetRPS, error);

        // Check if RPS is within range
        if (isRPSWithinTolerance(error)) {
            markAsDone();
            return;
        }

        // Calculate and set motor power
        double oldPower = currentPower;
        double newPower = calculateNewPower(error);
        shooter.setPower(newPower);

        Log.i("ShooterReady_PID", String.format(
            "PID Update: Error=%.2f, Integral=%.2f, Derivative=%.2f, Power: %.3f -> %.3f (Δ=%.3f)",
            error, integral, (error - previousError), oldPower, newPower, (newPower - oldPower)
        ));

        // Update state and transition
        updatePIDState(newPower, error);
        transitionToAccelerationState();
    }

    private double calculateAverageRPS() {
        return rpsSum / rpsReadCount;
    }

    private double getTargetRPS() {
        return shooter.getTargetRPS(SharedData.getOdometryPosition(), target);
    }

    private void logRPSReadings(double averageRPS, double targetRPS, double error) {
        KLog.d("ShooterReady_RPS", String.format(
            "Current: %.2f RPS (avg of %d readings) | Target: %.2f RPS | Error: %.2f RPS",
            averageRPS, rpsReadCount, targetRPS, error
        ));
    }

    private boolean isRPSWithinTolerance(double error) {
        return Math.abs(error) <= TARGET_RPS_TOLERANCE;
    }

    private void markAsDone() {
        isDone = true;
        KLog.d("ShooterReady", "RPS within tolerance - Ready!");
    }

    private double calculateNewPower(double error) {
        // Update integral with anti-windup
        integral += error;
        double maxIntegral = 100.0;
        if (integral > maxIntegral) integral = maxIntegral;
        if (integral < -maxIntegral) integral = -maxIntegral;

        // Calculate derivative
        double derivative = error - previousError;

        // Calculate PID output
        double pidOutput = (KP * error) + (KI * integral) + (KD * derivative);

        // Calculate new power
        double newPower = currentPower + pidOutput;

        // Clamp power to valid range
        if (newPower > MAX_POWER) newPower = MAX_POWER;
        if (newPower < MIN_POWER) newPower = MIN_POWER;

        return newPower;
    }

    private void updatePIDState(double newPower, double error) {
        currentPower = newPower;
        previousError = error;
    }

    private void transitionToAccelerationState() {
        currentState = State.WAITING_FOR_ACCELERATION;
        stateTimer.reset();
    }

    private void handleWaitingForAcceleration() {
        // Wait for motor to accelerate (150ms)
        if (stateTimer.milliseconds() >= MOTOR_ACCELERATION_WAIT_MS) {
            // Return to reading RPS
            currentState = State.READING_RPS;
            stateTimer.reset();
            rpsSum = 0;
            rpsReadCount = 0;
            KLog.d("ShooterReady", "Motor acceleration complete, reading RPS again");
        }
    }
}
