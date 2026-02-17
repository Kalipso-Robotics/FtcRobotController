package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {
    public static double timeToStabilize = 50; //50
    public static double kp = 0.085; //0.006; //TODO tune until going from depot to near is fast enough
    public static double kp_rampDown = 0.085; // 0.003
    public static double ki = 0; //.001;
    public static double kd = 0; //.012;
    public static double kf = 0.01147578609; //0.01205432614;
    public static double kA = 0;

    public static double accelBoostDeltaRPSThreshold = 3;
    public static double decelBoostDeltaRPSThreshold = -3;
    public static double SHOOTER_LOOKUP_TIME = 25;
    public static boolean hoodFlipDirection = true;
    public static boolean shouldShootOnTheMoveRPS = true;
}
