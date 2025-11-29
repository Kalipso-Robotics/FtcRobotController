package com.kalipsorobotics.modules.shooter;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;

@Config
public class ShooterInterpolationConfig {
    // 2025-11-28 14:31:19.233  Distance: 1104.3206911490881 params: ShooterParams(rps=30.95, hood=0.8000) GOOD
    // Distance: 1647.4467343437843 params: ShooterParams(rps=32.50, hood=0.8000) UNDER
    // Distance: 1390.6547891191399 params: ShooterParams(rps=32.65, hood=0.8000) UNDER
    public static double[] shooterData = {
            602.23, 28.57, 0.3,
            701.54, 30, 0.45,
            853.96, 27.86, 0.5,
            975.8, 29.29, 0.55,
            1024.07, 31.43, 0.6,
            1104, 30.7, 0.55, // FIRST SHOOT NEAR
            1143.96, 30.71, 0.55,
            1251.09, 32.14, 0.55,
            1290.48, 31.43, 0.55,
            1321.83, 30.71, 0.55,
            1341.74, 32.14, 0.55,
            1390, 33, 0.55,
            1411.42, 33.2, 0.55,
            1496.56, 33.4, 0.55,
            1496.59, 33.6, 0.55,
            1526.71, 33.8, 0.55,
            1595.31, 34.29, 0.55,  // a bit under shoot point 2 NEAR
            1647, 33, 0.55,
            1700.15, 33, 0.55,
            1813.77, 33, 0.55,
            1931.24, 34.29, 0.55,
            2070.17, 35, 0.55,
            2186.13, 37.86, 0.55,
            2186.2, 37.86, 0.55,
            2328.5, 38.57, 0.55,
            2328.51, 40.71, 0.55,
            2446.11, 40.8, 0.55, // overshoot tuned down 41.43 to 41.03 to 40.8
            2590.71, 42.14, 0.55,
            2729.31, 44.29, 0.55,
            2865.43, 44.29, 0.55,
            3001.9, 45, 0.55,
            3138.04, 45, 0.55,
            3283.8, 47.14, 0.55,
            3375, 47.2, 0.55, // second and third shoot depot, often has low battery and needs higher RPS
            3441.86, 46.5, 0.55,
            3495, 46.5, 0.55,
            3560, 46.6, 0.55, // First shoot depot, usually overshoot because of battery, so its lower to compensate
            3618, 46.6, 0.55,
            3768, 46.6, 0.55,
    };

}
