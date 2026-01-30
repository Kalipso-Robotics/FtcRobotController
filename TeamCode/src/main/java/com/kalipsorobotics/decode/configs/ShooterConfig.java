package com.kalipsorobotics.decode.configs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {
    public static double timeToStabilize = 50;
    public static double kp = 0.015; //0.06; //TODO tune until going from depot to near is fast enough
    public static double kp_rampDown = 0.03;
    public static double ki = 0; //.001;
    public static double kd = 0; //.012;
    public static double kf = 0.01142857143; //0.01205432614;
}
