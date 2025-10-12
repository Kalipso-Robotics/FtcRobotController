package com.kalipsorobotics.utilities;

import com.kalipsorobotics.PID.PIDController;
import com.kalipsorobotics.math.CalculateTickPer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Utility class for DC motors with RPS control
 * Provides smart power ramping to achieve target RPS quickly
 */
public class KMotor {
    private final DcMotorEx motor;

    // RPS tracking
    private double prevTicks = 0;
    private double prevTimeMS;

    private double prevRPS = 0;

    // PID controller for RPS control
    private final PIDController pidController;
    private double targetRPS = 0;

    // Power limits
    private static final double MAX_POWER = 1.0;
    private static final double MIN_POWER = 0.0;

    // Default PID constants - tune these values
    private static final double DEFAULT_KP = 0.0007;
    private static final double DEFAULT_KI = 0.0001;
    private static final double DEFAULT_KD = 0.0005;

    private final ElapsedTime elapsedTime;

    public KMotor(DcMotor motor) {
        this(motor, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
    }

    public KMotor(DcMotor motor, double kp, double ki, double kd) {
        this.motor = (DcMotorEx) motor;
        this.prevTicks = motor.getCurrentPosition();
        this.prevTimeMS = System.currentTimeMillis();
        this.elapsedTime = new ElapsedTime();
        this.pidController = new PIDController(kp, ki, kd, "KMotor_" + motor.getDeviceName());
    }

    /**
     * Calculate current RPS of the motor
     * @return current rotations per second
     */
    public double getRPS() {/*
        double currentTicks = motor.getCurrentPosition();
        double currentTimeMS = System.currentTimeMillis();
        double deltaTicks = Math.abs(currentTicks - prevTicks);
        double deltaRotation = CalculateTickPer.ticksToRotation6000RPM(deltaTicks);
        double deltaTimeMS = currentTimeMS - prevTimeMS;
        // Avoid division by zero
        if (deltaTimeMS < 10) {
            KLog.d("KMotor", "getRPS: Time delta too small (" + deltaTimeMS + "ms), returning previous RPS");
            return prevRPS;
        }

        KLog.d("KMotor", "getRPS: deltaTicks: " + deltaTicks + ", deltaRotation: " + deltaRotation + ", deltaTimeMS: " + deltaTimeMS);


        // If too much time has passed (>500ms), reset tracking and return 0
        // This prevents inaccurate readings when getRPS() hasn't been called for a while
        if (deltaTimeMS > 1000) {
            prevTicks = currentTicks;
            prevTimeMS = currentTimeMS;
            KLog.d("KMotor", "getRPS: Time delta too large (" + deltaTimeMS + "ms), resetting tracking");
            return CalculateTickPer.ticksToRotation6000RPM(motor.getVelocity());
        }



        double rps = (deltaRotation) / (deltaTimeMS / 1000.0);
        prevTicks = currentTicks;
        prevTimeMS = currentTimeMS;
        prevRPS = rps;
        return rps;*/
        double rps = CalculateTickPer.ticksToRotation6000RPM(motor.getVelocity());
        KLog.d("KMotor", "getRPS: " + rps);
        return rps;
    }

    /**
     * Set target RPS and adjust motor power to achieve it in the fastest way
     * This method calculates current RPS and adjusts power accordingly using PID control
     * Does not start from the beginning - uses current state for smart ramping
     *
     * @param targetRPS desired rotations per second
     */
    public void goToRPS(double targetRPS) {
       /* // Update target if changed significantly (>0.005 RPS)
        if (Math.abs(targetRPS - this.targetRPS) > 0.005) {
            this.targetRPS = targetRPS;
            // Reset PID on target change to prevent overshoot
            pidController.reset();
        }

        // Get current RPS
        double currentRPS = getRPS();

        // Calculate PID output
        double pidOutput = pidController.calculate(currentRPS, targetRPS);

        // Get current power and adjust
        double currentPower = motor.getPower();
        double newPower = currentPower + pidOutput;

        // Clamp power to valid range
        newPower = Math.max(MIN_POWER, Math.min(MAX_POWER, newPower));

        // Set new power
        motor.setPower(newPower);

        // Log for debugging
        KLog.d("KMotor", String.format(
            "Target: %.2f RPS, Current: %.2f RPS, Error: %.2f, PIDOutPut: %.2f, Power: %.3f -> %.3f",
            targetRPS, currentRPS, (targetRPS - currentRPS), pidOutput, currentPower, newPower
        ));*/
        double targetTPS = CalculateTickPer.rotationToTicks6000RPM(targetRPS);
        this.targetRPS = targetRPS;
        KLog.d("KMotor", "goToRPS: " + targetRPS);
        motor.setVelocity(targetTPS);
    }

    /**
     * Check if motor has reached target RPS within tolerance
     * @param tolerance acceptable error in RPS
     * @return true if current RPS is within tolerance of target
     */
    public boolean isAtTargetRPS(double tolerance) {
        double currentRPS = getRPS();
        KLog.d("KMotor", "Current RPS: " + currentRPS + " Target RPS: " + targetRPS);
        boolean isAtTarget = (Math.abs(currentRPS - targetRPS) <= tolerance);
        if (isAtTarget) {
            KLog.d("ShooterReady", "RPS within tolerance - Ready!");
        }
        return isAtTarget;
    }

    /**
     * Reset PID state (useful when starting a new control sequence)
     */
    public void resetPID() {
        pidController.reset();
        prevTicks = motor.getCurrentPosition();
        prevTimeMS = System.currentTimeMillis();
    }

    /**
     * Set motor power directly (bypasses RPS control)
     * @param power motor power (-1.0 to 1.0)
     */
    public void setPower(double power) {
        motor.setPower(power);
    }

    /**
     * Get current motor power
     * @return current power setting
     */
    public double getPower() {
        return motor.getPower();
    }

    /**
     * Stop the motor
     */
    public void stop() {
        motor.setPower(0);
        resetPID();
    }

    /**
     * Get the target RPS
     * @return target RPS
     */
    public double getTargetRPS() {
        return targetRPS;
    }

    /**
     * Get the PID controller for tuning
     * @return the PID controller
     */
    public PIDController getPIDController() {
        return pidController;
    }

    /**
     * Get the underlying DcMotor object
     * @return the motor
     */
    public DcMotor getMotor() {
        return motor;
    }

    /**
     * Set the motor's zero power behavior
     * @param behavior zero power behavior
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }

    /**
     * Set the motor's direction
     * @param direction motor direction
     */
    public void setDirection(DcMotor.Direction direction) {
        motor.setDirection(direction);
    }

    /**
     * Get current encoder position
     * @return encoder ticks
     */
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }
}
