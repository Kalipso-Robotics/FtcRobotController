package org.firstinspires.ftc.teamcode.kalipsorobotics.utilities;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig.DEFAULT_VOLTAGE;

import org.firstinspires.ftc.teamcode.kalipsorobotics.PID.PIDFController;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.CalculateTickPer;
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

    // PIDF controller for RPS control
    private final PIDFController pidfController;
    private double targetRPS = 0;

    private double targetTicks = 0;

    // Power limits
    private static final double MAX_POWER = 1.0;
    private static final double MIN_POWER = -1;
    private int ticksOffset = 0;

    // Default PIDF constants - tune these values for optimal RPS control
    // These values are designed to prevent overshoot while reaching target quickly
//    private static final double DEFAULT_KP = 0.05;   // Proportional - moderate correction
//    private static final double DEFAULT_KI = 0.0001; // Integral - small to prevent wind-up
//    private static final double DEFAULT_KD = 0.01;   // Derivative - strong to prevent overshoot
//    private static final double DEFAULT_KF = 0.01;   // Feedforward - base power per RPS



    private final ElapsedTime elapsedTime;

    public KMotor(DcMotor motor, double kp, double ki, double kd, double kf, double ks) {
        this.motor = (DcMotorEx) motor;
        this.prevTicks = motor.getCurrentPosition();
        this.prevTimeMS = System.currentTimeMillis();
        this.elapsedTime = new ElapsedTime();
        this.pidfController = new PIDFController(kp, ki, kd, kf, ks, "KMotor_" + motor.getDeviceName());
    }

    public KMotor(DcMotor motor, double kp, double ki, double kd, double kf) {
        this(motor, kp, ki, kd, kf, 0);
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
        KLog.d("KMotor", () -> "getRPS: " + rps);
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
        this.targetRPS = targetRPS;

        // Get current RPS
        double currentRPS = getRPS();

        double rawPower = pidfController.calculate(currentRPS, targetRPS, 0.0, MAX_POWER);
        double newPower = rawPower;
        double voltage = SharedData.getVoltage();
//        if (!(Math.abs(voltage) < 0.000000001)) {
//            newPower = clampPower(rawPower * (DEFAULT_VOLTAGE / voltage));
//        }

        KLog.d("VoltageCompensation", () -> "Raw Power: " + rawPower +
                " Compensated Power: " + newPower +
                " Current Voltage: " + voltage
                );
        // Set new power
        motor.setPower(newPower);

        // Log for debugging
        KLog.d("KMotor", () -> String.format(
            "Target: %.2f RPS, Current: %.2f RPS, Error: %.2f, Power: %.3f, MinPower: %.3f, MaxPower: %.3f",
            targetRPS, currentRPS, (targetRPS - currentRPS), newPower, 0.0, MAX_POWER
        ));
    }

    public double clampPower(double power, double max_power, double min_power) {
        return Math.max(min_power, Math.min(power, max_power));
    }
    public double clampPower(double power) {
        return clampPower(power, 1, -1);
    }
    public void goToTargetTicks(int targetTicks) {
        this.targetTicks = targetTicks;

        int currentTicks = motor.getCurrentPosition();
        // Error = where we want to be - where we are
        int error = targetTicks - currentTicks;
        double newPower = clampPower(pidfController.calculate(error));
        motor.setPower(newPower);
        KLog.d("KMotor", () -> String.format(
                "Target: %d Ticks, Current: %d Ticks, Error: %d, Power: %.3f, MinPower: %.3f, MaxPower: %.3f",
                targetTicks, currentTicks, error, newPower, -1.0, 1.0
        ));
    }


    /**
     * Check if motor has reached target RPS within tolerance
     * @param tolerance acceptable error in RPS
     * @return true if current RPS is within tolerance of target
     */
    public boolean isAtTargetRPS(double tolerance) {
        double currentRPS = getRPS();
        KLog.d("KMotor", () -> "Current RPS: " + currentRPS + " Target RPS: " + targetRPS);
        boolean isAtTarget = (Math.abs(currentRPS - targetRPS) < tolerance); // changed <= to < (11/21)
        if (isAtTarget) {
            KLog.d("ShooterReady", "RPS within tolerance - Ready!");
        }

        return isAtTarget;
    }

    /**
     * Reset PIDF state (useful when starting a new control sequence)
     */
    public void resetPID() {
        pidfController.reset();
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
    public void stopAndResetPID() {
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
     * Get the PIDF controller for tuning
     * @return the PIDF controller
     */
    public PIDFController getPIDFController() {
        return pidfController;
    }

    /**
     * Get the underlying DcMotor object
     * @return the motor
     */
    public DcMotorEx getMotor() {
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
        return motor.getCurrentPosition() + ticksOffset;
    }

    public void setTicksOffset(int ticksOffset) {
        this.ticksOffset = ticksOffset;
    }
}
