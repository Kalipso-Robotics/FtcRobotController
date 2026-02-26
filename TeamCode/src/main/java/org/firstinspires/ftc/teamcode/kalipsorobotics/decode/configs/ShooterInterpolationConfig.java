package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;


import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterInterpolationConfig {
    public static final double FALLBACK_OUT_OF_RANGE_DISTANCE_MM = 3460;
    public static double rpsOffset = 0;
    public static double hoodOffset = 0;
    public static final double HOOD_INIT_OFFSET = 0;
    public final static double MIN_HOOD = 0.1284; //0.72 old hood
    public final static double MAX_HOOD = 0.3; // 0.3 old hood 37.38mm space by caliper from bottom
    public static double FAR_SHOOT_RPS = 66.5;
    public static final double MAX_RPS = 73;

    public static final double NEAR_SHOOT_RPS = 36.6;
    public static final int FAR_DISTANCE = 2760;
    public static final int NEAR_DISTANCE = 1785;
    public static double maxHoodCompensate = 0.005; //0.02166;
    public static double minHoodCompensate = -maxHoodCompensate;//-0.02166;

    public static double hoodCompensateCoefficient = maxHoodCompensate; // 0.2

    public static  double DEFAULT_VOLTAGE = 12.5;
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

    static double compensatedMaxHood = MAX_HOOD - maxHoodCompensate; //0.31754;
    private static final double rpsTune = 0.25;
    public static final int BETWEEN_FAR_NEAR_TIP = 3240;
    public static double[] shooterData = {
            810, 45.4 + rpsTune, MIN_HOOD,
            1050, 45.4 + rpsTune, compensatedMaxHood - 0.085,
            1310, 47.7 + rpsTune, compensatedMaxHood - 0.085, //roughly first shot for near auto
            1518, 49.6 + rpsTune, compensatedMaxHood - 0.05,
            1705, 51.6 + rpsTune, compensatedMaxHood - 0.05,
            1898, 52.8 + rpsTune, compensatedMaxHood - 0.02,
            2075, 54.7 + rpsTune, compensatedMaxHood,
            2244, 56.6 + rpsTune, compensatedMaxHood,
            2417, 57.6 + rpsTune, compensatedMaxHood,
            2561, 60 + rpsTune, compensatedMaxHood,
            2769, 61.8 + rpsTune, compensatedMaxHood,
            2974, 63.7 + rpsTune, compensatedMaxHood,
            3154, 65 + rpsTune, compensatedMaxHood,
            BETWEEN_FAR_NEAR_TIP, 66.2 + rpsTune + 0.2, compensatedMaxHood,
            3460, 67.3 + rpsTune + 0.2, compensatedMaxHood,
            3700, 70.0 + rpsTune + 0.2, compensatedMaxHood,
            4202, MAX_RPS + rpsTune + 0.2, compensatedMaxHood,
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
