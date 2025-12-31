package com.kalipsorobotics.actions.turret;

import com.acmerobotics.dashboard.config.Config;
import com.kalipsorobotics.modules.Turret;

@Config
public class TurretConfig { //

    public static double X_INIT_SETUP_MM = 3419.5; //3400      3,619.5 mm - 200m = 3,419.5
    public static double Y_INIT_SETUP_MM = 1400; // 1413.55      1,413.55 = 1,219.2 mm + 194.35
    public static int TICKS_INIT_OFFSET = 0;
    public static double DEFAULT_TOLERANCE_TICKS = (Turret.TICKS_PER_DEGREE) * 1.5;
    public static double kP = 0.008;   // faster response
    public static double kI = 0;       // keep at 0
    public static double kD = 0.0001;  // reduces overshoot (main time saver) 0.0002
    public static double kS = 0;    // faster final approach
    public static double kF = 225;     // 375 // for tracking moving targets

    public static double kP_teleop = 0.006;   //0.01 faster response
    public static double kI_teleop = 0;       // keep at 0
    public static double kD_teleop = 0.0008;  // reduces overshoot (main time saver) 0.0006
    public static double kS_teleop = 0;   //0.04 faster final approach
    public static double kF_teleop = 225;     // 375 // for tracking moving targets


}
