package com.kalipsorobotics.modules.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {
    public static double timeToStabilize = 0;
    public static double kp = 0.000363;
    public static double ki = 0.000003;
    public static double kd = 0.000016;
    public static double kf = 0.01218297;
    public static double kfBase = -0.00886369;
    public static double maintainTimeOutMS = 1000;
    //  kp=0.000190 ki=0.000003 kd=0.000028 BEST VALUES from Iterative Learning
    //  kp=0.000363 ki=0.000003 kd=0.000016 BEST VALUES 2
}
