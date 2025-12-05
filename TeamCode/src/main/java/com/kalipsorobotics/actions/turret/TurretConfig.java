package com.kalipsorobotics.actions.turret;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretConfig { //

    public static double X_INIT_SETUP_MM = 3357; //3400
    public static double Y_INIT_SETUP_MM = 1350; // 1400
    public static double TICKS_INIT_OFFSET = 0;
    public static double kP = 0.0021;
    public static double kI = 0.00003; //0.000030
    public static double kD = 0.000165; //0.000090
    public static double kS = 0.15; // static base

}
