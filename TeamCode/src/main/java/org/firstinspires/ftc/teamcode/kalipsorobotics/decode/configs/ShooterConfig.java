package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

@Config
public class ShooterConfig {

    public static final double OUT_OF_RANGE_THRESHOLD = 20 * 12 * 25.4;
    // Hive Cell aim points, measured. SharedData.getTargetPoint() picks one by robot y.
    public static final Point TARGET_POINT_1 = new Point(1466.796, 2189.771); // Cell A (high y)
    public static final Point TARGET_POINT_2 = new Point(1466.797, 1391.947); // Cell B (low y)
    /** Old DECODE goal, fixed. Only the decode/ OpModes use it; BioBuzz aims at SharedData.getTargetPoint(). */
    public static final Point TARGET_POINT = TARGET_POINT_1;

    // ---- Hive Cell target switch (SharedData.getTargetPoint) ----
    /** Robot y above this aims at TARGET_POINT_1 (Cell A), below it at TARGET_POINT_2 (Cell B). */
    public static double TARGET_SWITCH_Y_MM = 1828.8; // Hive center (field 3657.6 / 2)
    /** Inside this band around TARGET_SWITCH_Y_MM the last Cell is kept, so the target does not flicker. TODO-measure */
    public static double TARGET_SWITCH_DEADBAND_MM = 150;
    /** The target is re-checked once every this many odometry updates, not every loop. */
    public static int TARGET_CHECK_EVERY_N_LOOPS = 4;
    public static double timeToStabilize = 50; // TODO-measure
    public static double kp = 0.085; //TODO tune until going from depot to near is fast enough
    public static double kp_rampDown = 0.085; // 0.003
    public static double ki = 0;
    public static double kd = 0;
    // kf = steady power / steady RPS at about 0.8 power on a full battery, measured with MotorPowerRPSDataCollector.
    public static double kf = 0.01148; // TODO-measure
    public static double kA = 0;
    public static boolean shouldBangBangCompensate = false;

    public static double accelBoostDeltaRPSThreshold = 3; // TODO-measure
    public static double decelBoostDeltaRPSThreshold = -3; // TODO-measure
    // Accel power when bang-bang compensation is on: full past NEAR_DISTANCE, lower when near.
    public static double BANG_POWER_FAR = 1; // TODO-measure
    public static double BANG_POWER_NEAR = 0.7; // TODO-measure
    public static boolean hoodFlipDirection = true;
    /** Flywheel motor direction; Shooter and MotorPowerRPSDataCollector both use it. */
    public static DcMotorSimple.Direction SHOOTER_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE; // TODO-measure

    public static double AUTO_SHOOTER_READY_TIMEOUT_MS = 3500; // TODO-measure
    // TeleOp never fires on timeout: RPS or turret not ready by then aborts the shot (flywheel keeps spinning).
    public static double TELEOP_READY_TIMEOUT_MS = 3000; // TODO-measure
    /** Re-zero odometry x/y to the Limelight after every shot; false turns it off everywhere. */
    public static boolean REZERO_AFTER_EACH_SHOT = true;

    // Weak pass shot, GP1 Y. Placeholders below the shortest distance-table shot.
    public static double PASS_RPS = 25; // TODO-measure
    public static double PASS_HOOD = 0.2; // TODO-measure

    // Scale shooter power by DEFAULT_VOLTAGE / battery voltage in KMotor.goToRPS.
    public static boolean USE_VOLTAGE_COMPENSATION = true;
}