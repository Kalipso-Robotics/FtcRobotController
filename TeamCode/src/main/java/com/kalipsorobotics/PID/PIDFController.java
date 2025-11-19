package com.kalipsorobotics.PID;

import android.os.SystemClock;
import androidx.annotation.NonNull;

import com.kalipsorobotics.utilities.KLog;

/**
 * PIDF Controller for velocity control (e.g., motor RPS)
 * Adds feedforward term to standard PID for better setpoint tracking
 */
public class PIDFController {

    private double kp;
    private double ki;
    private double kd;
    private double kf;
    private double kfBase;

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
    public PIDFController(double P, double I, double D, double F, double baseF, String controllerName) {
        kp = P;
        ki = I;
        kd = D;
        kf = F;
        this.kfBase = baseF;
        integralError = 0;
        lastError = 0;
        lastTime = SystemClock.elapsedRealtimeNanos();

        name = controllerName;
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
     * @return Control output to apply
     */
    public double calculate(double current, double target) {
        double error = target - current;

        double currentTime = SystemClock.elapsedRealtimeNanos();
        double timeDelta = (currentTime - lastTime) / 1e9;

        // Accumulate integral error
        integralError += error * timeDelta;

        // Calculate PID components
        double proportional = kp * error;
        double integral = ki * integralError;
        double derivative = kd * (error - lastError) / timeDelta;

        // Feedforward component - base output for target velocity
        double feedforward = kf * target + kfBase;

        // Calculate total output
        double output = proportional + integral + derivative + feedforward;

        // Log calculated values
        KLog.d("PIDF_" + name, String.format(
            "Current: %.2f | Target: %.2f | Error: %.2f | P: %.4f | I: %.4f | D: %.4f | F: %.4f | Output: %.4f",
            current, target, error, proportional, integral, derivative, feedforward, output));

        lastTime = currentTime;
        lastError = error;

        return output;
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

        lastTime = currentTime;
        lastError = error;

        // No feedforward when only error is provided
        return proportional + integral + derivative;
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
