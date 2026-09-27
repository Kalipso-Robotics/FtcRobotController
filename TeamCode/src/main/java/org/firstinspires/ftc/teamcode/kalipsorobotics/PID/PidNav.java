package org.firstinspires.ftc.teamcode.kalipsorobotics.PID;

import android.os.SystemClock;

import androidx.annotation.NonNull;

public class PidNav {
    private double P;
    private double I;
    private double D;
    private double S;

    private static final double MAX_UPDATE_GAP_MS = 500;

    private double errorIntegral = 0;
    private double lastTime = System.currentTimeMillis();
    private double lastError = 0;

    public PidNav(double P, double I, double D, double S){
        this.P = P;
        this.I = I;
        this.D = D;
        this.S = S;
    }

    public PidNav(double P, double I, double D) {
        this(P, I, D, 0);
    }

    public double getPower(double error){
        double currentTime = System.currentTimeMillis();
        double deltaTime = currentTime - lastTime;

        // lastTime starts at construction, and a path point's PID can be built at init and first
        // used many seconds later. Integrating over that gap winds up the I term so much that it
        // holds the robot tens of degrees off target. Treat a long gap as a fresh start instead.
        if (deltaTime > MAX_UPDATE_GAP_MS) {
            errorIntegral = 0;
            deltaTime = 0;
        }

        errorIntegral += error * (deltaTime/100);
        double errorDerivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
        double staticBase = S * Math.signum(error);

        lastTime = currentTime;
        lastError = error;
        return (P * error) + (I * errorIntegral) + (D * errorDerivative) + staticBase;
    }

    public double getP() {
        return P;
    }
    public double getI() {
        return I;
    }
    public double getD() {
        return D;
    }
    public double getS() {
        return S;
    }
    public void setP(double p) {
        P = p;
    }
    public void setI(double i) {
        I = i;
    }
    public void setD(double d) {
        D = d;
    }
    public void setS(double s) {
        this.S = s;
    }

    public void setErrorIntegral(double errorIntegral) {
        this.errorIntegral = errorIntegral;
    }

    @Override
    public String toString() {
        return "PidNav{" +
                "D=" + D +
                ", I=" + I +
                ", P=" + P +
                '}';
    }
}