package com.kalipsorobotics.decode.configs;




public class ShooterInterpolationConfig {
    public static double rpsOffset = 0;
    public static double hoodOffset = 0;
    public static final double HOOD_INIT_OFFSET = 0;
    public final static double MIN_HOOD = 0.1226; //0.72 old hood
    public final static double MAX_HOOD = 0.3392; // 0.3 old hood 37.38mm space by caliper from bottom
    public static double FAR_SHOOT_RPS = 66.5;
    public static final double NEAR_SHOOT_RPS = 36.6;
    public static final int FAR_DISTANCE = 2760;
    public static final int NEAR_DISTANCE = 1785;
    public static double minHoodCompensate = -0.02166;
    public static double maxHoodCompensate = 0.02166;
    public static double hoodCompensateCoefficient = maxHoodCompensate; // 0.2
    /*
    public static double[] shooterData0 = {
            710, 33.2 + rpsHack, MIN_HOOD,
            890, 34.3 + rpsHack, MIN_HOOD - hoodHack,
            1115, 35.0 + rpsHack, MAX_HOOD - hoodHack, // Near Auto First shoot, tuned, near auto distance: 1037.1287983659502
            1320, 36.2 + rpsHack, MAX_HOOD - hoodHack,
            1630, 37.8 + rpsHack, MAX_HOOD - hoodHack, //Near Auto, Trip 1,2 shooting point, tuned, distance: 1631.7190444436199
            NEAR_DISTANCE, 39.1 + rpsHack, MAX_HOOD - hoodHack, //Near Tip Point
            1920, 39.8 + rpsHack, MAX_HOOD - hoodHack,
            2050, 40.7 + rpsHack, MAX_HOOD - hoodHack,
            2200, 41.7 + rpsHack, MAX_HOOD - hoodHack, // 41.0
            2400, 41.9 + rpsHack, MAX_HOOD - hoodHack,
            2570, 45.2 + rpsHack, MAX_HOOD - hoodHack,
            FAR_DISTANCE, 45.7 + rpsHack, MAX_HOOD - hoodHack, // TIP AREA DON'T TOUCH, far shoot
            2965, 47.4 + rpsHack, MAX_HOOD - hoodHack, // TIP AREA !!! very good like 80% accuracy DON'T TOUCH. far shoot
            3095, 48.0 + rpsHack, MAX_HOOD - hoodHack, // FAR SHOOT, first shoot ( 0 , 0 )
            3300, 48.6 + rpsHack, MAX_HOOD - hoodHack,
            3500, 49.3 + rpsHack, MAX_HOOD - hoodHack,
    };
    */

    static double compensatedMaxHood = MAX_HOOD - maxHoodCompensate;
    public static double[] shooterData = {
            810, 46.2, compensatedMaxHood - 0.16,
            1050, 46.2, compensatedMaxHood - 0.04,
            1310, 47.8, compensatedMaxHood - 0.12,
            1518, 47.8, compensatedMaxHood - 0.1,
            1705, 50.7, compensatedMaxHood - 0.08,
            1898, 53.7, compensatedMaxHood - 0.04,
            2075, 56.4, compensatedMaxHood,
            2244, 56.7, compensatedMaxHood,
            2417, 57.4, compensatedMaxHood,
            2561, 61.7, compensatedMaxHood,
            2769, 60.5, compensatedMaxHood,
            2974, 65.95, compensatedMaxHood,
            3154, 65, compensatedMaxHood,
            3240, 65.7, compensatedMaxHood,
            3260, 65.7, compensatedMaxHood,
            3460, 67.8, compensatedMaxHood,
            3700, 70.0, compensatedMaxHood,
            4202, 74.1, compensatedMaxHood,
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
