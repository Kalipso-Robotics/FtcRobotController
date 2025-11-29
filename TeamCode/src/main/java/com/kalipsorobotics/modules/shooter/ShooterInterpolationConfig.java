package com.kalipsorobotics.modules.shooter;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;

@Config
public class ShooterInterpolationConfig {
    public static double[] shooterData = {
            602.23, 28.57, 0.3,
            701.54, 30, 0.45,
            853.96, 27.86, 0.5,
            975.8, 29.29, 0.55,
            1024.07, 31.43, 0.6,
            1104, 31.5, 0.55, // FIRST SHOOT NEAR
            1143.96, 32, 0.55,
            1251.09, 32.4, 0.55,
            1290.48, 32.43, 0.55,
            1321.83, 32.5, 0.55,
            1341.74, 32.6, 0.55,
            1390, 32.6, 0.55,
            1411.42, 33.2, 0.55,
            1469, 33.5, 0.55,
            1496.59, 33.6, 0.55,
            1526.71, 33.8, 0.55,
            1595.31, 33.4, 0.55,  // a bit under shoot point 2 NEAR (34.29mm)
            1629, 33, 0.55,
            1647, 33.5, 0.55,
            1700.15, 34, 0.55,
            1813.77, 34, 0.55,
            1931.24, 34.7, 0.55,
            1984, 35, 0.55,
            2070.17, 35.5, 0.55,
            2186.13, 37.86, 0.55,
            2186.2, 37.86, 0.55,
            2328.5, 38.57, 0.55,
            2328.51, 40.71, 0.55,
            2446.11, 40.71, 0.55,
            2590.71, 42.14, 0.55,
            2729.31, 43.3, 0.55,
            2865.43, 44.3, 0.55,
            3001.9, 45, 0.55,
            3138.04, 45, 0.55,
            3283.8, 45.7, 0.55,
            3375, 46.6, 0.55, // second and third shoot depot, often has low battery and needs higher RPS
            3441.86, 46.6, 0.55,
            3495, 46.5, 0.55,
            3560, 46.6, 0.55, // First shoot depot, usually overshoot because of battery, so its lower to compensate
            3618, 46.6, 0.55,
            3768, 46.6, 0.55,
    };

}
