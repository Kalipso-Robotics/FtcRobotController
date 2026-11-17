package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {

    public static final double OUT_OF_RANGE_THRESHOLD = 20 * 12 * 25.4;
    public static double timeToStabilize = 50; //50
    public static double kp = 0.085; //TODO tune until going from depot to near is fast enough
    public static double kp_rampDown = 0.085; // 0.003
    public static double ki = 0;
    public static double kd = 0;
    // kf = steady power / steady RPS at about 0.8 power on a full battery, measured with MotorPowerRPSDataCollector.
    public static double kf = 0.01148; // TODO-measure
    public static double kA = 0;

    public static boolean shouldBangBangCompensate = false;

    public static double accelBoostDeltaRPSThreshold = 3;
    public static double decelBoostDeltaRPSThreshold = -3;
    public static boolean hoodFlipDirection = true;

    public static double AUTO_SHOOTER_READY_TIMEOUT_MS = 3500;
    // TeleOp never fires on timeout: RPS or turret not ready by then aborts the shot (flywheel keeps spinning).
    public static double TELEOP_READY_TIMEOUT_MS = 3000;

    // Weak pass shot, GP1 Y. Placeholders below the shortest distance-table shot.
    public static double PASS_RPS = 25; // TODO-measure
    public static double PASS_HOOD = 0.2; // TODO-measure

    // Scale shooter power by DEFAULT_VOLTAGE / battery voltage in KMotor.goToRPS.
    public static boolean USE_VOLTAGE_COMPENSATION = true;
}