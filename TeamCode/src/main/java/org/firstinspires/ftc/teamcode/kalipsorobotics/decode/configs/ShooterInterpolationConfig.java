package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;


import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterInterpolationConfig {
    public static final double FALLBACK_OUT_OF_RANGE_DISTANCE_MM = 3460;
    public static double rpsOffset = 0;
    public static double hoodOffset = 0;
    public static final double HOOD_INIT_OFFSET = 0;
    // LUT hood range. The mechanical hard limits are ShooterConfig.HOOD_DOWN_POS / HOOD_UP_POS.
    public final static double MIN_HOOD = 0; // TODO-measure //0.72 old hood
    public final static double MAX_HOOD = 0.5; // TODO-measure // 0.3 old hood 37.38mm space by caliper from bottom
    public static double FAR_SHOOT_RPS = 66.5; // TODO-measure
    // One motor now spins 6 flywheels + 4 weights: the ceiling is about 1/kf (~87 RPS loaded), and the
    // PIDF needs headroom to recover between balls and under NECTAR voltage sag. Shooter clamps every target to this.
    public static final double MAX_RPS = 70; // TODO-measure
    // Used when shooterData is empty so the shooter still spins to something sane.
    public static final double DEFAULT_RPS = 40; // TODO-measure

    // Per-ball-type offsets. Unused until the robot can tell POLLEN from NECTAR.
    public static double POLLEN_RPS_OFFSET = 0;
    public static double POLLEN_HOOD_OFFSET = 0;
    public static double NECTAR_RPS_OFFSET = 0;
    public static double NECTAR_HOOD_OFFSET = 0;

    public static final double NEAR_SHOOT_RPS = 36.6; // TODO-measure
    public static final int FAR_DISTANCE = 2760; // TODO-measure
    public static final int NEAR_DISTANCE = 1785; // TODO-measure
    public static double maxHoodCompensate = 0.005; // TODO-measure //0.02166;
    public static double minHoodCompensate = -maxHoodCompensate;//-0.02166;

    public static double hoodCompensateCoefficient = maxHoodCompensate; // TODO-measure // 0.2

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
    public static final int BETWEEN_FAR_NEAR_TIP = 3240; // TODO-measure
    // Rows of: distanceMM, rps, hoodPosition, intakeRPS (sorted by distance). Empty until measured on the BIOBUZZ robot;
    // ShooterInterpolationDataLookup returns DEFAULT_RPS / MIN_HOOD while it is empty.
    // TODO-measure: fill the table; the commented-out rows above are DECODE carry-over, do not reuse them.
    public static double[] shooterData = {
            950,  35.8, 0.345, 10.0, // 10
            1070, 36.2, 0.322, 9.0,  // 9
            1200, 37.0, 0.285, 7.0,  // 7
            1350, 36.5, 0.28,  8.0,  // 8
            1500, 38.0, 0.255, 13.0, // 13
            1686, 39.0, 0.25,  13.0, // 13
    };
}
