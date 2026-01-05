package com.kalipsorobotics.modules.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterInterpolationConfig {

    public static double rpsOffset = 0;
    public static double hoodOffset = 0;
    public static final double HOOD_INIT_OFFSET = 0;
    public final static double MAX_HOOD = 0.72; //0.64 old hood
    public final static double MIN_HOOD = 0.3; // 37.38mm space by caliper from bottom
    public static double MAX_RPS = 55;
    public static double FAR_SHOOT_RPS = 46.6;
    public static final double NEAR_SHOOT_RPS = 36.6;
    public static double[] shooterData = {
        710, 31.0, MIN_HOOD,
        890, 31.7, 0.55,
        1115, 34.0, 0.67, // Near Auto First shoot, tuned, near auto distance: 1037.1287983659502
        1320, 35.4, MAX_HOOD,
        1630, 36.3, MAX_HOOD, //Near Auto, Trip 1,2 shooting point, tuned, distance: 1631.7190444436199
        1785, 36.6, MAX_HOOD, //Near auto, trip 3 shooting point, tuned, distance: 1736.5272931917884
        1920, 38.8, MAX_HOOD,
        2050, 39.7, MAX_HOOD,
        2200, 41.0, MAX_HOOD,
        2400, 41.3, MAX_HOOD,
        2570, 44.5, MAX_HOOD,
        2760, 45.5, MAX_HOOD, // TIP AREA DON'T TOUCH, far shoot
        2965, 46.6, MAX_HOOD, // TIP AREA !!! very good like 80% accuracy DON'T TOUCH. far shoot
        3095, 48.2, MAX_HOOD, // FAR SHOOT, first shoot ( 0 , 0 )
        3300, 48.7, MAX_HOOD,
        3500, 49.3, MAX_HOOD,
    };

    /*
    public static double[] shooterData = {

        3600, 55, MAX_HOOD,
        3300, 53, MAX_HOOD,
        3100, 51.4, MAX_HOOD,
        2940, 50, MAX_HOOD,
        2890, 50, MAX_HOOD,
        2750, 48.5, MAX_HOOD,
        2600, 47.2, NORMAL_HOOD,
        2420, 45, NORMAL_HOOD,
        2310, 44, NORMAL_HOOD,
        2180, 44, NORMAL_HOOD,
        2050, 43.2, NORMAL_HOOD,
        1990, 42.2, NORMAL_HOOD,
        1900, 41.5, NORMAL_HOOD,
        1800, 41, NORMAL_HOOD,
        1670, 40.9, NORMAL_HOOD,
        1560, 40.2, NORMAL_HOOD,
        1450, 38.5, NORMAL_HOOD,
        1350, 37.8, NORMAL_HOOD,
        1245, 37.1, 0.71,
        1130, 35.5, 0.62,
        1060, 34.5, 0.62, // funny
        1000, 34.2, 0.62,
        915, 33.7, 0.62,
        602.23, 28.57 + 1, 0,
        724, 32 + 1, 0,
    }; */


   // Odometry distance base;
//    public static double[] shooterData = {
//            602.23, 28.57 - 0.5, 0.4,
//            724, 32 - 0.5, 0.4,
//            918, 33.4 - 0.5, 0.4,
//            1126, 32 - 0.5, 0.4,
//            1143.96, 31.5 - 0.5, 0.4, // FIRST SHOOT NEAR,
//            1251.09, 32.14, MAX_HOOD,
//            1290.48, 32.43, MAX_HOOD,
//            1321.83, 32.71, MAX_HOOD,
//            1341.74, 33.00 + 0.5, MAX_HOOD,
//            1411.42, 33.86 + 0.5, MAX_HOOD,
//            1496.59, 34.86 + 1.5, MAX_HOOD,
//            1526.71, 35.29 + 1.5, MAX_HOOD,
//            1595.31, 35.5 + 1.5,  MAX_HOOD, // SECOND SHOOT NEAR,
//            1700.15, 35.54 + 1.5, MAX_HOOD,
//            1813.77, 35.86 + 1.5, MAX_HOOD,
//            1931.24, 35.29, MAX_HOOD,
//            2070.17, 36.39, MAX_HOOD,
//            2186.13, 37.86, MAX_HOOD,
//            2186.2, 37.86, MAX_HOOD,
//            2328.5, 38.57, MAX_HOOD,
//            2328.51, 40.71, MAX_HOOD,
//            2446.11, 41, MAX_HOOD,
//            2590.71, 42.14, MAX_HOOD,
//            2729.31, 44.29, MAX_HOOD,
//            2865.43, 43.29, MAX_HOOD,
//            3001.9, 45 + 1, MAX_HOOD, // good
//            3138.04, 44.5 + 1.3, MAX_HOOD, // good
//            3283.8, 46 + 1.3, MAX_HOOD,
//            3441.86, 46.6 + 1.3, MAX_HOOD,
//    };
    /* Before manually tuning near
    public static double[] shooterData = {
            602.23, 28.57 + 1, 0.2,
            724, 32 + 1, 0,
            918, 33.4 + 1, 0.3,
            1126, 32 + 1.5, 0.4,
            1143.96, 31.5 + 2, 0.5, // FIRST SHOOT NEAR,
            1251.09, 32.14 + 2, MAX_HOOD,
            1290.48, 31.43 + 2, MAX_HOOD,
            1321.83, 30.71 + 2, MAX_HOOD,
            1341.74, 32.14 + 2, MAX_HOOD,
            1411.42, 32.86 + 2, MAX_HOOD,
            1496.56, 30 + 2, MAX_HOOD,
            1496.59, 32.86 + 2, MAX_HOOD,
            1526.71, 34.29 + 2, MAX_HOOD,
            1595.31, NEAR_SHOOT_RPS + 2, MAX_HOOD, // SECOND SHOOT NEAR,
            1700.15, 32.14 + 2, MAX_HOOD,
            1813.77, 32.86 + 2, MAX_HOOD,
            1931.24, 34.29 + 2, MAX_HOOD,
            2070.17, 35 + 2, MAX_HOOD,
            2186.13, 37.86 + 2, MAX_HOOD,
            2186.2, 37.86 + 2, MAX_HOOD,
            2328.5, 38.57 + 2, MAX_HOOD,
            2328.51, 40.71 + 2, MAX_HOOD,
            2446.11, 41 + 2, MAX_HOOD,
            2590.71, 42.14 + 3, MAX_HOOD,
            2729.31, 44.29 + 3, MAX_HOOD,
            2865.43, 43.29 + 4, MAX_HOOD,
            3001.9, 45 + 4.5, MAX_HOOD, // good
            3138.04, 44.5 + 5.5, MAX_HOOD, // good
            3283.8, 46 + 6.5, MAX_HOOD,
            3441.86, 46.6 + 7.5, MAX_HOOD,
    };
     */

    public static double[] getMinValue() {
        return new double[] {shooterData[1], shooterData[2]};
    }
    public static double[] getFarShoot() {
        return new double[] {FAR_SHOOT_RPS, MAX_HOOD};
    }
//    public static double TEST_FORCE_SHOOT_RPS = 48;
//    public static double TEST_FORCE_SHOOT_HOOD = 0.57;
//    public static double[] getMaxValue() {
//        return new double[] {TEST_FORCE_SHOOT_RPS, TEST_FORCE_SHOOT_HOOD};
//    }
    public static double[] getNearValue() {
        return new double[] {NEAR_SHOOT_RPS, MAX_HOOD};
    }

}