package com.kalipsorobotics.decode.configs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {
    public static double timeToStabilize = 50;
    public static double kp = 0.085; //0.006; //TODO tune until going from depot to near is fast enough
    public static double kp_rampDown = 0.085; // 0.003
    public static double ki = 0; //.001;
    public static double kd = 0; //.012;
    public static double kf = 0.01142857143; //0.01205432614;
    public static double kA = 0;

    public static double hoodHack = 0.2;
    public static double hoodCompensateCoefficient = 0.2;
    public static double rpsHack = 1.5;
    public static double minHoodCompensate = -0.2;
    public static double maxHoodCompensate = 0.2;
}
