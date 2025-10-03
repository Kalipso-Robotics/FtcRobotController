package com.kalipsorobotics.math;

/** Quick solver: constant wheel RPM + flat arc → hood extension length. */
public class CalculateHoodLength {

    // ---- Required inputs (set these to your real values) ----
    private final Position goalPosition;           // goal center in field frame (meters)
    private final double wheelDiameter_m = 0.1016; // 4" wheel
    private final double slipEta = 0.85;           // ball/wheel slip (0.80..0.90 typical)
    private final double wheelRpmConst = 6000.0;   // your constant wheel RPM (use measured!)
    private final double gravity = 9.80665;        // m/s^2

    // Heights (meters)
    private final double releaseHeight_m = 0.46;   // ball center at release
    private final double goalHeight_m    = 0.9845; // from manual; 1.00 is okay too

    // Hood mechanical limits (deg) — used for safety clamp
    private final double hoodMinDeg = 0.0;
    private final double hoodMaxDeg = 35.0;

    // ---- Hood angle ↔ extension calibration (fill these!) ----
    // Measure these once with a digital angle gauge and a ruler.
    // eMin/eMax are your actuator/hood extension limits (meters of travel).
    private final double eMin_m = 0.00;     // TODO: measure
    private final double eMax_m = 0.08;     // TODO: measure
    private final double thetaAtEMinDeg = 0.0;   // hood angle when e=eMin (deg)
    private final double thetaAtEMaxDeg = 35.0;  // hood angle when e=eMax (deg)

    public CalculateHoodLength(Position goalPosition) {
        this.goalPosition = goalPosition;
    }

    /** Call this each loop: returns hood extension length (meters) for the given robot pose. */
    public double calculateHoodLengthFlatArc(Position robotPosition) {
        double r = horizontalRange(robotPosition, goalPosition);
        double deltaH = goalHeight_m - releaseHeight_m;

        // 1) exit speed from constant wheel RPM
        double exitSpeed = slipEta * (Math.PI * wheelDiameter_m * wheelRpmConst / 60.0); // m/s

        // 2) flat-arc angle from speed (in radians)
        double thetaRad = angleFlatFromSpeed(r, deltaH, exitSpeed);

        // Safety clamp to hood limits
        double thetaDeg = clamp(Math.toDegrees(thetaRad), hoodMinDeg, hoodMaxDeg);

        // 3) convert angle → hood extension (meters)
        return extensionFromAngle(thetaDeg);
    }

    /** If you ever want HIGH ARC instead, call this and use angleHighFromSpeed below. */
    public double calculateHoodLengthHighArc(Position robotPosition) {
        double r = horizontalRange(robotPosition, goalPosition);
        double deltaH = goalHeight_m - releaseHeight_m;
        double exitSpeed = slipEta * (Math.PI * wheelDiameter_m * wheelRpmConst / 60.0);
        double thetaRad = angleHighFromSpeed(r, deltaH, exitSpeed); // <-- high-arc branch
        double thetaDeg = clamp(Math.toDegrees(thetaRad), hoodMinDeg, hoodMaxDeg);
        return extensionFromAngle(thetaDeg);
    }

    // ---------------------- Math helpers ----------------------

    /** Flat-arc branch of the closed-form angle-from-speed solution. */
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

    /** High-arc branch (not used for your current request, but here for completeness). */
    private double angleHighFromSpeed(double r, double deltaH, double v) {
        if (r <= 0) throw new IllegalArgumentException("Range must be positive");
        double v2 = v * v;
        double disc = v2 * v2 - gravity * (gravity * r * r + 2.0 * deltaH * v2);
        if (disc < 0) throw new IllegalStateException("Speed too low for this range/height");
        double root = Math.sqrt(disc);
        double tanHigh = (v2 + root) / (gravity * r);    // <-- HIGH ARC (plus)
        double theta = Math.atan(tanHigh);
        if (r * Math.tan(theta) <= deltaH)
            throw new IllegalStateException("Geometry invalid: r*tan(theta) <= Δh");
        return theta;
    }

    /** Linear angle→extension mapping; replace with spline linkage is nonlinear. */
    private double extensionFromAngle(double thetaDeg) {
        // Normalize angle over calibrated range
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
