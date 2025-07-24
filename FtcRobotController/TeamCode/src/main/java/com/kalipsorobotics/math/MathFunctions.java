package com.kalipsorobotics.math;


import android.util.Log;

public class MathFunctions {
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
        return a*a;
    }

    public static double interpolateAngle(double start, double end, double t) {
        double delta = angleWrapRad(end - start);
        return start + delta * t;
    }
}
