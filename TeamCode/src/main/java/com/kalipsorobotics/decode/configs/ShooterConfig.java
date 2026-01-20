package com.kalipsorobotics.decode.configs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {
    public static double timeToStabilize = 50;
    public static double kp_accel = 0.015;
    public static double ki_accel = 0.001;
    public static double kd_accel = 0.012;
    public static double kf_accel = 0.01205432614;

    public static double kp_decel = 0.006;
    public static double ki_decel = 0.0005;
    public static double kd_decel = 0.003;
    public static double kf_decel = 0.01205432614;



}
