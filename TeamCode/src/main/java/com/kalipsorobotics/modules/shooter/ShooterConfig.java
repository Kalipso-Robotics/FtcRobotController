package com.kalipsorobotics.modules.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {
    public static double timeToStabilize = 0;
    // PID: this combo is working well; leave it
    public static double kp = 0.014000;
    public static double ki = 0.001000;
    public static double kd = 0.002000;

    public static double kf = 0.009338;   // <-- Global optimum



    public static double maintainTimeOutMS = 1000;
    //  kp=0.000190 ki=0.000003 kd=0.000028 BEST VALUES from Iterative Learning
    //  kp=0.000363 ki=0.000003 kd=0.000016 BEST VALUES 2
    //  Learning complete! Best PID: kp=0.000010 ki=0.001000 kd=0.001000 (cost: 59.36)
    //Learning complete! Best PID: kp=0.010000 ki=0.001000 kd=0.001000 (cost: 54.27)
}
