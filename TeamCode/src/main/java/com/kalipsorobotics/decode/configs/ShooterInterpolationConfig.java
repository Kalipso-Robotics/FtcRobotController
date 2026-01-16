package com.kalipsorobotics.decode.configs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterInterpolationConfig {

    public static double rpsOffset = 0;
    public static double hoodOffset = 0;
    public static double HOOD_INIT_OFFSET = 0;
    public static double MAX_HOOD = 0.72; //0.64 old hood
    public static double MIN_HOOD = 0.3; // 37.38mm space by caliper from bottom
    public static double FAR_SHOOT_RPS = 46.6;
    public static double NEAR_SHOOT_RPS = 36.6;
    public static int FAR_DISTANCE = 2760;
    public static int NEAR_DISTANCE = 1785;
    public static double[] shooterData = {
        730, 31.6, MIN_HOOD,
        890, 32.0, 0.55,
        1115, 33.9, 0.67, // Near Auto First shoot, tuned, near auto distance: 1037.1287983659502
        1320, 36.1, MAX_HOOD,
        1630, 37.1, MAX_HOOD, //Near Auto, Trip 1,2 shooting point, tuned, distance: 1631.7190444436199
        NEAR_DISTANCE, 36.6, MAX_HOOD, //Near auto, trip 3 shooting point, tuned, distance: 1736.5272931917884
        1920, 38.8, MAX_HOOD,
        2050, 39.7, MAX_HOOD,
        2200, 41.0, MAX_HOOD,
        2400, 41.3, MAX_HOOD,
        2570, 44.5, MAX_HOOD,
        FAR_DISTANCE, 45.5, MAX_HOOD, // TIP AREA DON'T TOUCH, far shoot
        2965, 46.6, MAX_HOOD, // TIP AREA !!! very good like 80% accuracy DON'T TOUCH. far shoot
        3095, 48.2, MAX_HOOD, // FAR SHOOT, first shoot ( 0 , 0 )
        3300, 48.7, MAX_HOOD,
        3500, 49.3, MAX_HOOD,
    };


    public static double[] getMinValue() {
        return new double[] {shooterData[1], shooterData[2]};
    }
    public static double[] getFarShoot() {
        return new double[] {FAR_SHOOT_RPS, MAX_HOOD};
    }
    public static double[] getNearValue() {
        return new double[] {NEAR_SHOOT_RPS, MAX_HOOD};
    }

}