package com.kalipsorobotics.modules.shooter;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;

@Config
public class ShooterInterpolationConfig {
    public static double[] shooterData = {
            602.23, 28.57, 0.25,
            724, 32, 0.25,
            918, 33.4, 0.35,
            1126, 32, 0.45,
            1143.96, 31.5, 0.8, // FIRST SHOOT NEAR,
            1251.09, 32.14, 0.8,
            1290.48, 31.43, 0.8,
            1321.83, 30.71, 0.8,
            1341.74, 32.14, 0.8,
            1411.42, 32.86, 0.8,
            1496.56, 30, 0.8,
            1496.59, 32.86, 0.8,
            1526.71, 34.29, 0.8,
            1595.31, 32.86, 0.8, // SECOND SHOOT NEAR,
            1700.15, 32.14, 0.8,
            1813.77, 32.86, 0.8,
            1931.24, 34.29, 0.8,
            2070.17, 35, 0.8,
            2186.13, 37.86, 0.8,
            2186.2, 37.86, 0.8,
            2328.5, 38.57, 0.8,
            2328.51, 40.71, 0.8,
            2446.11, 41, 0.8,
            2590.71, 42.14, 0.8,
            2729.31, 44.29, 0.8,
            2865.43, 43.29, 0.8,
            3001.9, 45, 0.8,
            3138.04, 44.5, 0.8,
            3283.8, 46, 0.8, // 2 + 3 SHOOT FAR,
            3441.86, 46.6, 0.8, // FIRST SHOOT FAR,
    };

    public static double[] getMaxValue() {
        return new double[] {46.6, 0.8};
    }

}
