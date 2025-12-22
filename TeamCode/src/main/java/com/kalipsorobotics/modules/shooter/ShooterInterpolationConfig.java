package com.kalipsorobotics.modules.shooter;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;

@Config
public class ShooterInterpolationConfig {
    public final static double MAX_HOOD = 0.71;
    public static final double NEAR_SHOOT_RPS = 32.86;
    public static double[] shooterData = {
            602.23, 28.57, 0,
            724, 32, 0,
            918, 33.4, 0.1,
            1126, 32, 0.2,
            1143.96, 31.5, MAX_HOOD, // FIRST SHOOT NEAR,
            1251.09, 32.14, MAX_HOOD,
            1290.48, 31.43, MAX_HOOD,
            1321.83, 30.71, MAX_HOOD,
            1341.74, 32.14, MAX_HOOD,
            1411.42, 32.86, MAX_HOOD,
            1496.56, 30, MAX_HOOD,
            1496.59, 32.86, MAX_HOOD,
            1526.71, 34.29, MAX_HOOD,
            1595.31, NEAR_SHOOT_RPS, MAX_HOOD, // SECOND SHOOT NEAR,
            1700.15, 32.14, MAX_HOOD,
            1813.77, 32.86, MAX_HOOD,
            1931.24, 34.29, MAX_HOOD,
            2070.17, 35, MAX_HOOD,
            2186.13, 37.86, MAX_HOOD,
            2186.2, 37.86, MAX_HOOD,
            2328.5, 38.57, MAX_HOOD,
            2328.51, 40.71, MAX_HOOD,
            2446.11, 41, MAX_HOOD,
            2590.71, 42.14, MAX_HOOD,
            2729.31, 44.29, MAX_HOOD,
            2865.43, 43.29, MAX_HOOD,
            3001.9, 45, MAX_HOOD,
            3138.04, 44.5, MAX_HOOD,
            3283.8, 46, MAX_HOOD, // 2 + 3 SHOOT FAR,
            3441.86, 46.6, MAX_HOOD, // FIRST SHOOT FAR,
    };

    public static double[] getMaxValue() {
        return new double[] {shooterData[shooterData.length - 2], shooterData[shooterData.length - 1]};
    }
    public static double[] getNearValue() {
        return new double[] {NEAR_SHOOT_RPS, MAX_HOOD};
    }

}
