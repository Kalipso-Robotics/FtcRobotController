package com.kalipsorobotics.math;


public class MathFunctions {

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static double angleWrapRad(double angle) {
        angle = angle % (2 * Math.PI);
        if (angle > Math.PI) {
            angle -= 2 * Math.PI;
        } else if (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    public static double angleWrapDeg(double angle) {
        return Math.toDegrees(angleWrapRad(Math.toRadians(angle)));
    }

    /**
     * Wraps angle only when it exceeds ±190 degrees (hysteresis band).
     * This prevents oscillation near the ±180 boundary.
     */
    public static double angleWrapDegHysteresis(double angle) {
        return Math.toDegrees(angleWrapRadHysteresis(Math.toRadians(angle)));
    }

    /**
     * Wraps angle only when it exceeds ±(π + 10°) radians (hysteresis band).
     * This prevents oscillation near the ±π boundary.
     */
    public static double angleWrapRadHysteresis(double angle) {
        double threshold = Math.PI + Math.toRadians(15); // π + 10° in radians
        angle = angle % (2 * Math.PI);
        if (angle > threshold) {
            angle -= 2 * Math.PI;
        } else if (angle < -threshold) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    public static double angleWrapRadForce0ToPi(double angle) {
        angle = angle % (2 * Math.PI);
        if (angle < 0) {
            angle += 2 * Math.PI;
        }
        if (angle > Math.PI) {
            angle = 2 * Math.PI - angle;
        }
        return angle;
    }

    public static double maxAbsValueDouble(double a, double... others) {

        double max = a;

        for (double next : others) {
            if (Math.abs(next) > Math.abs(max)) {
                max = next;
            }
        }

        return Math.abs(max);
    }

    public static double minAbsValueDouble(double a, double... others) {

        double min = a;

        for (double next : others) {
            if (Math.abs(next) < Math.abs(min)) {
                min = next;
            }
        }

        return Math.abs(min);
    }

    public static double square(double a) {
        return a * a;
    }

    public static double interpolateAngle(double start, double end, double t) {
        double delta = angleWrapRad(end - start);
        return start + delta * t;
    }

    public static double distance(Point p1, Point p2) {
        return Math.hypot(p2.getX() - p1.getX(), p2.getY() - p1.getY());
    }
}
