package com.kalipsorobotics.actions.turret;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretConfig { //

    public static double X_INIT_SETUP_MM = 3357; //3400
    public static double Y_INIT_SETUP_MM = 1350; // 1400
    public static double TICKS_INIT_OFFSET = 0;
    public static double kP = 0.0025; // increased for faster convergence // was 0.00565
    public static double kI = 0; //0.00003; //0.000030
    public static double kD = 0; // reduced to prevent oscillation // was 0.0002
    public static double kS = 0.05; // reduced for smoother low-power movement // was 0.1
    public static double kF = 375; // velocity feedforward gain (power per rad/ms)
}
