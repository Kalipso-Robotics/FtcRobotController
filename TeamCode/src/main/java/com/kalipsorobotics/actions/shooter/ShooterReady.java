package com.kalipsorobotics.actions.shooter;

import android.util.Log;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterReady extends Action {

    public static final Point FAR_LAUNCH_POINT = new Point(3600, 1200);

    private final Shooter shooter;
    private final Point target;

    // PID constants - tune these values
    private static final double KP = 0.001;  // Proportional gain
    private static final double KI = 0.0001;  // Integral gain
    private static final double KD = 0.0005;  // Derivative gain

    private static final double TARGET_RPS_TOLERANCE = 3.0;
    private static final double STABLE_TIME_MS = 200; // Time RPS must be stable
    private static final double UPDATE_INTERVAL_MS = 500; // Update interval for RPS reading and power adjustment
    private static final double MAX_POWER = 1.0;
    private static final double MIN_POWER = 0.0;

    // PID state
    private double previousError = 0;
    private double integral = 0;
    private double currentPower = MIN_POWER;

    // Stabilization tracking
    private ElapsedTime stableTimer = null;
    private ElapsedTime updateTimer = null;
    private boolean isStable = false;

    public ShooterReady(Shooter shooter, Point target) {
        this.shooter = shooter;
        this.target = target;
        this.name = "ShooterReady";
    }

    @Override
    protected boolean checkDoneCondition() {
        return isDone;
    }

    @Override
    public void update() {
        if (!hasStarted) {
            hasStarted = true;
            previousError = 0;
            integral = 0;
            currentPower = MIN_POWER;
            stableTimer = null;
            updateTimer = new ElapsedTime();
            isStable = false;
        }

        // Always update hood position based on current position
        shooter.updateHoodFromPosition(SharedData.getOdometryPosition(), target);

        // Only update RPS and power every UPDATE_INTERVAL_MS
        if (updateTimer.milliseconds() < UPDATE_INTERVAL_MS) {
            return;
        }

        // Reset update timer for next interval
        updateTimer.reset();

        double targetRPS = shooter.getTargetRPS(SharedData.getOdometryPosition(), target);
        double currentRPS = shooter.getRPS();

        // Calculate error
        double error = targetRPS - currentRPS;

        // PID calculations
        integral += error;

        // Anti-windup: clamp integral to prevent excessive buildup
        double maxIntegral = 100.0;
        if (integral > maxIntegral) integral = maxIntegral;
        if (integral < -maxIntegral) integral = -maxIntegral;

        double derivative = error - previousError;

        // Calculate PID output
        double pidOutput = (KP * error) + (KI * integral) + (KD * derivative);

        // Update power based on PID output
        currentPower += pidOutput;

        // Clamp power to valid range
        if (currentPower > MAX_POWER) currentPower = MAX_POWER;
        if (currentPower < MIN_POWER) currentPower = MIN_POWER;

        // Set motor power
        shooter.setPower(currentPower);

        // Store error for next iteration
        previousError = error;

        Log.d("ShooterReady", "Target RPS: " + targetRPS + ", Current RPS: " + currentRPS +
              ", Error: " + error + ", Power: " + currentPower + ", PID: " + pidOutput);

        // Check if RPS is within tolerance
        if (Math.abs(error) <= TARGET_RPS_TOLERANCE) {
            // Start or continue stability timer
            if (stableTimer == null) {
                stableTimer = new ElapsedTime();
                Log.d("ShooterReady", "Started stability timer");
            } else if (stableTimer.milliseconds() >= STABLE_TIME_MS) {
                // RPS has been stable for required time
                isDone = true;
                Log.d("ShooterReady", "RPS stable and ready!");
                return;
            }
        } else {
            // RPS moved out of tolerance, reset stability timer
            stableTimer = null;
        }
    }
}
