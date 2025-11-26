package com.kalipsorobotics.actions.turret;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretConfig { //

    public static double X_INIT_SETUP_MM = 3250 + 150; //121 inches

    public static double Y_INIT_SETUP_MM = 1050 + 340; //46inches 1000
    public static double kP = 0.0021;
    public static double kI = 0.000030;
    public static double kD = 0.000090;

}
