package com.kalipsorobotics.math;

/** Quick solver: constant wheel RPM + flat arc → hood extension length. */
public class CalculateHoodLength {

    private final Position goalPosition;
    private final double wheelDiameter_m = 0.1016;
    private final double slipEta = 0.85;
    private final double wheelRpmConst = 6000.0;
    private final double gravity = 9.80665;

    private final double releaseHeight_m = 0.46;
    private final double goalHeight_m    = 0.9845;

    private final double hoodMinDeg = 0.0;
    private final double hoodMaxDeg = 35.0;

    private final double eMin_m = 0.00;     // TODO: measure
    private final double eMax_m = 0.08;     // TODO: measure
    private final double thetaAtEMinDeg = 0.0;
    private final double thetaAtEMaxDeg = 35.0;

    public CalculateHoodLength(Position goalPosition) {
        this.goalPosition = goalPosition;
    }

    //flat arc
    public double calculateHoodLengthFlatArc(Position robotPosition) {
        double r = horizontalRange(robotPosition, goalPosition);
        double deltaH = goalHeight_m - releaseHeight_m;

        double exitSpeed = slipEta * (Math.PI * wheelDiameter_m * wheelRpmConst / 60.0); // m/s

        double thetaRad = angleFlatFromSpeed(r, deltaH, exitSpeed);

        double thetaDeg = clamp(Math.toDegrees(thetaRad), hoodMinDeg, hoodMaxDeg);

        return extensionFromAngle(thetaDeg);
    }
    //high arc
    public double calculateHoodLengthHighArc(Position robotPosition) {
        double r = horizontalRange(robotPosition, goalPosition);
        double deltaH = goalHeight_m - releaseHeight_m;
        double exitSpeed = slipEta * (Math.PI * wheelDiameter_m * wheelRpmConst / 60.0);
        double thetaRad = angleHighFromSpeed(r, deltaH, exitSpeed); // <-- high-arc branch
        double thetaDeg = clamp(Math.toDegrees(thetaRad), hoodMinDeg, hoodMaxDeg);
        return extensionFromAngle(thetaDeg);
    }

    private double angleFlatFromSpeed(double r, double deltaH, double v) {
        if (r <= 0) throw new IllegalArgumentException("Range must be positive");
        double v2 = v * v;
        double disc = v2 * v2 - gravity * (gravity * r * r + 2.0 * deltaH * v2);
        if (disc < 0) throw new IllegalStateException("Speed too low for this range/height");
        double root = Math.sqrt(disc);
        double tanFlat = (v2 - root) / (gravity * r);    // <-- FLAT ARC (minus)
        double theta = Math.atan(tanFlat);
        if (r * Math.tan(theta) <= deltaH)
            throw new IllegalStateException("Geometry invalid: r*tan(theta) <= Δh");
        return theta;
    }

    private double angleHighFromSpeed(double r, double deltaH, double v) {
        if (r <= 0) throw new IllegalArgumentException("Range must be positive");
        double v2 = v * v;
        double disc = v2 * v2 - gravity * (gravity * r * r + 2.0 * deltaH * v2);
        if (disc < 0) throw new IllegalStateException("Speed too low for this range/height");
        double root = Math.sqrt(disc);
        double tanHigh = (v2 + root) / (gravity * r);
        double theta = Math.atan(tanHigh);
        if (r * Math.tan(theta) <= deltaH)
            throw new IllegalStateException("Geometry invalid: r*tan(theta) <= Δh");
        return theta;
    }

    private double extensionFromAngle(double thetaDeg) {
        double t = (thetaDeg - thetaAtEMinDeg) / (thetaAtEMaxDeg - thetaAtEMinDeg);
        t = clamp(t, 0.0, 1.0);
        return eMin_m + t * (eMax_m - eMin_m);
    }

    private static double horizontalRange(Position robot, Position goal) {
        double dx = goal.getX() - robot.getX();
        double dy = goal.getY() - robot.getY();
        return Math.hypot(dx, dy);
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}
