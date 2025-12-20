package com.kalipsorobotics.PID;

import android.os.SystemClock;
import androidx.annotation.NonNull;

import com.kalipsorobotics.utilities.KLog;

import org.opencv.core.Mat;

/**
 * PIDF Controller for velocity control (e.g., motor RPS)
 * Adds feedforward term to standard PID for better setpoint tracking
 */
public class PIDFController {

    private double kp;
    private double ki;
    private double kd;
    private double kf;
    private double ks;
    private double integralError;
    private double lastError;
    private double lastTime;

    private final String name;

    /**
     * Create a PIDF controller
     * @param P Proportional gain - corrects current error
     * @param I Integral gain - eliminates steady-state error
     * @param D Derivative gain - dampens oscillations and reduces overshoot
     * @param F Feedforward gain - provides base output proportional to target
     * @param controllerName Name for identification
     */
    public PIDFController(double P, double I, double D, double F, double S, String controllerName) {
        kp = P;
        ki = I;
        kd = D;
        kf = F;
        ks = S;
        integralError = 0;
        lastError = 0;
        lastTime = SystemClock.elapsedRealtimeNanos();

        name = controllerName;
    }

    public PIDFController(double P, double I, double D, double F, String controllerName) {
        this(P, I, D, F, 0, controllerName);
    }

    /**
     * Reset the controller state
     * Call this when starting a new control sequence
     */
    public void reset() {
        integralError = 0;
        lastError = 0;
        lastTime = SystemClock.elapsedRealtimeNanos();
    }

    /**
     * Calculate control output based on current and target values
     * @param current Current process variable (e.g., current RPS)
     * @param target Target setpoint (e.g., target RPS)
     * @param minOutput Minimum allowed output (for saturation)
     * @param maxOutput Maximum allowed output (for saturation)
     * @return Control output clamped to [minOutput, maxOutput]
     */
    public double calculate(double current, double target, double minOutput, double maxOutput) {
        double error = target - current;

        double currentTime = SystemClock.elapsedRealtimeNanos();
        double timeDelta = (currentTime - lastTime) / 1e9;

        if (Math.abs(error) <= Math.abs(target * 0.1)){
            // Accumulate integral error
            integralError += error * timeDelta;
        }
        // Calculate PID components
        double proportional = kp * error;
        double integral = ki * integralError;
        double derivative = kd * (error - lastError) / timeDelta;

        // Feedforward component - base output for target velocity
        double feedforward = kf * target;

        // Clamp integral to ±10% of feedforward to prevent windup
        double maxIntegral = Math.abs(feedforward * 0.1);
        integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));

        double totalRawOutput = proportional + integral + derivative + feedforward;

        // Clamp output to valid range
        double clampedOutput = Math.max(minOutput, Math.min(maxOutput, totalRawOutput));

        // Log calculated values
        KLog.d("PIDF_" + name, String.format(
            "Current: %.2f | Target: %.2f | Error: %.2f | P: %.4f | I: %.4f | D: %.4f | F: %.4f | RawOutput: %.4f | Output: %.4f",
            current, target, error, proportional, integral, derivative, feedforward, totalRawOutput, clampedOutput));

        KLog.d("PIDF_" + name, String.format(
                "Current: %.2f | Target: %.2f | Error: %.2f | kP: %.4f | kI: %.4f | kD: %.4f | kF: %.4f | DeltaT: %.4f",
                current, target, error, kp, ki, kd, kf, timeDelta));

        lastTime = currentTime;
        lastError = error;

        return clampedOutput;
    }

    /**
     * Calculate control output based on current and target values (no saturation limits)
     * WARNING: Without saturation limits, anti-windup cannot be applied!
     * @param current Current process variable (e.g., current RPS)
     * @param target Target setpoint (e.g., target RPS)
     * @return Control output (unclamped)
     */
    public double calculate(double current, double target) {
        // Call the main method with no limits (effectively no clamping)
        return calculate(current, target, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    /**
     * Calculate control output based on error only
     * Note: This doesn't use feedforward since target is unknown
     * @param error Error value (target - current)
     * @return Control output to apply
     */
    public double calculate(double error) {
        double currentTime = SystemClock.elapsedRealtimeNanos();
        double timeDelta = (currentTime - lastTime) / 1e9;

        integralError += error * timeDelta;

        double proportional = kp * error;
        double integral = ki * integralError;
        double derivative = kd * (error - lastError) / timeDelta;
        double staticPower = ks * Math.signum(error);

        double output = proportional + integral + derivative;

        //clamp to make staticPower min
        output = Math.max(Math.abs(staticPower), Math.abs(output)) * Math.signum(output);

        lastTime = currentTime;
        lastError = error;

        // No feedforward when only error is provided
        return output;
    }

    // Tuning methods
    public double chKp(double delta) {
        return kp += delta;
    }

    public double chKi(double delta) {
        return ki += delta;
    }

    public double chKd(double delta) {
        return kd += delta;
    }

    public double chKf(double delta) {
        return kf += delta;
    }

    public double setKp(double val) {
        return kp = val;
    }

    public double setKi(double val) {
        return ki = val;
    }

    public double setKd(double val) {
        return kd = val;
    }

    public double setKf(double val) {
        return kf = val;
    }

    // Getters
    public double getKp() {
        return kp;
    }

    public double getKi() {
        return ki;
    }

    public double getKd() {
        return kd;
    }

    public double getKf() {
        return kf;
    }

    public String getName() {
        return name;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format("%s with Kp %f, Ki %f, Kd %f, Kf %f", name, kp, ki, kd, kf);
    }
}
