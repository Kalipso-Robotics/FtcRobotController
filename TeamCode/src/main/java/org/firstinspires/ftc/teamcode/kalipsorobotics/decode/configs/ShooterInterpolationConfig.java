package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;


import static org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig.FEED_RPS;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;

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

    static double compensatedMaxHood = MAX_HOOD - maxHoodCompensate; //0.31754;
    public static final int BETWEEN_FAR_NEAR_TIP = 3240; // TODO-measure
    // Rows of: distanceMM, rps, hoodPosition, intakeRPS (sorted by distance). Empty until measured on the BIOBUZZ robot;
    // ShooterInterpolationDataLookup returns DEFAULT_RPS / MIN_HOOD while it is empty.
    // TODO-measure: fill the table; the commented-out rows above are DECODE carry-over, do not reuse them.

    static double adjustShooterRPS = 0;

    //Distance, ShooterRPS, HoodPos, IntakeRPS
    public static double[] shooterData = {
            950,  34.5 + adjustShooterRPS, 0.425, FEED_RPS,
            1100, 37 + adjustShooterRPS, 0.420, FEED_RPS,
            1250, 39 + adjustShooterRPS, 0.305, FEED_RPS,
            1400, 39.5 + adjustShooterRPS, 0.295,  FEED_RPS,
            1540, 39.5 + adjustShooterRPS, 0.29, FEED_RPS,
            1702, 40.5 + adjustShooterRPS, 0.280,  FEED_RPS,
//            1862, 39 + adjustShooterRPS, 0.285, FEED_RPS
            2251, 39 + adjustShooterRPS, 0.285, FEED_RPS
    };
}
