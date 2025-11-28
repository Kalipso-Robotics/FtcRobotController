package com.kalipsorobotics.modules.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {
    public static double timeToStabilize = 500;
    public static double MAX_RPS = 55;

    // PID: this combo is working well; leave it
//    public static double kp = 0.014000 ;
//    public static double ki = 0.005000;
//    public static double kd = 0.002000;

    // Recommended starting point (no algorithm changes)
    public static double kp = 0.01;
    public static double ki = 0.001;
    public static double kd = 0.005;
    public static double kf = 0.01205432614;   // <-- Global optimum (fallback, not used with lookup table) 0.009338 OLD

    // Discrete RPS breakpoints from shooter tests
//    private static final double[] SHOOTER_RPS_POINTS = {
//        20, 22, 24, 26, 28, 30,
//        32, 34, 36, 38, 40, 42,
//        44, 46, 48, 50, 52, 54
//    };

    // Per-RPS kF values derived from all data (smoothed)
//    private static final double[] SHOOTER_KF_POINTS = {
//        0.00757 * 1.3, 0.00784 * 1.3, 0.00809 * 1.3, 0.00719 * 1.3, 0.00748 * 1.3, 0.00751 * 1.3,
//        0.00755 * 1.3, 0.00765 * 1.3, 0.00767 * 1.3, 0.00770 * 1.3, 0.00801 * 1.3, 0.00835 * 1.3,
//        0.00907 * 1.1, 0.00933 * 1.1, 0.00950 * 1.1, 0.00970 * 1.1, 0.00985 * 1.1, 0.00983 * 1.1
//    };

    /**
     * Get optimal kF value for a given target RPS using lookup table
     * @param targetRps target rotations per second
     * @return optimal kF value for the given RPS
     */
//    public static double getShooterKf(double targetRps) {
//        // Below lowest test point
//        if (targetRps <= SHOOTER_RPS_POINTS[0]) {
//            return SHOOTER_KF_POINTS[0];
//        }
//        // Between points: linear interpolation
//        for (int i = 0; i < SHOOTER_RPS_POINTS.length - 1; i++) {
//            double r0 = SHOOTER_RPS_POINTS[i];
//            double r1 = SHOOTER_RPS_POINTS[i + 1];
//
//            if (targetRps <= r1) {
//                double k0 = SHOOTER_KF_POINTS[i];
//                double k1 = SHOOTER_KF_POINTS[i + 1];
//
//                double t = (targetRps - r0) / (r1 - r0); // 0..1
//                return k0 + t * (k1 - k0);
//            }
//        }
//        // Above highest test point
//        return SHOOTER_KF_POINTS[SHOOTER_KF_POINTS.length - 1];
//    }

    public static double getShooterKf(double targetRps) {
        return kf;
    }
    public static double maintainTimeOutMS = 1000;
    //  kp=0.000190 ki=0.000003 kd=0.000028 BEST VALUES from Iterative Learning
    //  kp=0.000363 ki=0.000003 kd=0.000016 BEST VALUES 2
    //  Learning complete! Best PID: kp=0.000010 ki=0.001000 kd=0.001000 (cost: 59.36)
    //Learning complete! Best PID: kp=0.010000 ki=0.001000 kd=0.001000 (cost: 54.27)
}
