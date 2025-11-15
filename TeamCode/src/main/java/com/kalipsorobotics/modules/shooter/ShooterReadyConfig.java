package com.kalipsorobotics.modules.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterReadyConfig {
    public static double kp = 0.0007;
    public static double ki = 0.0001;
    public static double kd = 0.0005;

//    public static double kf = 0.025;
    public static double targetRPS = 46.0;
}
