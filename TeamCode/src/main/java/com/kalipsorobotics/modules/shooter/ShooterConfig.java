package com.kalipsorobotics.modules.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {
    public static double timeToStabilize = 0;
    public static double kp = 0.0003;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0.025;

    public static double minPowerScaleFactor = 1.0/100.0;

    public static double maintainTimeOutMS = 1000;
}
