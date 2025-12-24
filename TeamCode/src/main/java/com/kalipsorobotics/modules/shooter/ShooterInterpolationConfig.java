package com.kalipsorobotics.modules.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterInterpolationConfig {

    public final static double MAX_HOOD = 0.8;
    public final static double NORMAL_HOOD = 0.75;
    public static final double NEAR_SHOOT_RPS = 32.86  + 0.5;
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
    public static double[] shooterData = {
            602.23, 28.57 + 1, 0,
            724, 32 + 1, 0,
            918, 33.4 + 1, 0.1,
            1126, 32 + 1.5, 0.2,
            1143.96, 31.5 + 1.5, MAX_HOOD, // FIRST SHOOT NEAR,
            1251.09, 32.14 + 1.5, MAX_HOOD,
            1290.48, 31.43 + 1.5, MAX_HOOD,
            1321.83, 30.71 + 2, MAX_HOOD,
            1341.74, 32.14 + 1.5, MAX_HOOD,
            1411.42, 32.86 + 1.5, MAX_HOOD,
            1496.56, 30 + 2, MAX_HOOD,
            1496.59, 32.86 + 1.5, MAX_HOOD,
            1526.71, 34.29 + 1.5, MAX_HOOD,
            1595.31 + 1.5, NEAR_SHOOT_RPS, MAX_HOOD, // SECOND SHOOT NEAR,
            1700.15, 32.14 + 1.5, MAX_HOOD,
            1813.77, 32.86 + 1.5, MAX_HOOD,
            1931.24, 34.29 + 1, MAX_HOOD,
            2070.17, 35 + 1 + 0.5, MAX_HOOD,
            2186.13, 37.86 + 1 + 0.5, MAX_HOOD,
            2186.2, 37.86 + 1 + 0.5, MAX_HOOD,
            2328.5, 38.57 + 1 + 0.5, MAX_HOOD,
            2328.51, 40.71 + 1 + 0.5, MAX_HOOD,
            2446.11, 41 + 1 + 0.5, MAX_HOOD,
            2590.71, 42.14 + 1 + 0.5, MAX_HOOD,
            2729.31, 44.29 + 1 + 0.5, MAX_HOOD,
            2865.43, 43.29 + 1 + 0.5, MAX_HOOD,
            3001.9, 45 + 1 + 0.5, MAX_HOOD,
            3138.04, 44.5 + 1 + 0.5, MAX_HOOD,
            3283.8, 46 + 1 + 0.5, MAX_HOOD, // 2 + 3 SHOOT FAR,
            3441.86, 46.6 + 1 + 0.5, MAX_HOOD, // FIRST SHOOT FAR,
    };

    public static double[] getMaxValue() {
        return new double[] {shooterData[shooterData.length - 2], shooterData[shooterData.length - 1]};
    }
    public static double[] getNearValue() {
        return new double[] {NEAR_SHOOT_RPS, NORMAL_HOOD};
    }

}