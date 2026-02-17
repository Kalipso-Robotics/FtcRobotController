package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;

import com.acmerobotics.dashboard.config.Config;


@Config
public class TurretConfig {
    //==========TURRET=========
    public static final double TICKS_PER_ROTATION = 384.5;
    public static final double BIG_TO_SMALL_PULLEY = 125.0/32.0;
    public static final double TICKS_PER_DEGREE = (TICKS_PER_ROTATION * BIG_TO_SMALL_PULLEY) / 360.0;
    public static final double TICKS_PER_RADIAN = (TICKS_PER_ROTATION * BIG_TO_SMALL_PULLEY) / (2 * Math.PI); //

    //target MidPoint
    public static double X_INIT_SETUP_MM = 3419.5 - 152.2; //3400      3,619.5 mm - 200m = 3,419.5       142.5 - 8
    public static double Y_INIT_SETUP_MM = 1400 - 175; // 1400 - 177.8
    public static int TICKS_INIT_OFFSET = 0;
    public static double LOOK_AHEAD_TIME_MS = 50;
    public static double DEFAULT_TOLERANCE_TICKS = (TICKS_PER_DEGREE) * 1.5;
    public static double kP = 0.006;   // faster response
    public static double kI = 0;       // keep at 0
    public static double kD = 0.0004;  // reduces overshoot (main time saver) 0.0001
    public static double kS = 0.01;    // 0.025 faster final approach
    public static double kF = 0;     // 375 // for tracking moving targets
    public static double kP_teleop = 0.006;   //0.01 faster response
    public static double kI_teleop = 0;       // keep at 0
    public static double kD_teleop = 0.0004;  // reduces overshoot (main time saver) 0.0002
    public static double kS_teleop = 0.01;   //0.025 faster final approach
    public static double kF_teleop = 0;     // 375 // for tracking moving targets

    public static boolean shouldShootOnTheMoveTurret = true;
}
