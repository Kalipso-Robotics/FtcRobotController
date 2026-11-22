package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

import com.acmerobotics.dashboard.config.Config;


@Config
public class TurretConfig {
    //==========TURRET=========
    // One servo, no encoder. Servo 50T -> 15T pinion, same shaft 37T -> 92T turret ring (from CAD).
    // Two external meshes, so the turret turns the same direction as the servo.
    public static final double SERVO_GEAR_T = 50;
    public static final double PINION_T = 15;
    public static final double TOP_GEAR_T = 37;
    public static final double RING_T = 92;
    public static final double TURRET_DEG_PER_SERVO_DEG = (SERVO_GEAR_T / PINION_T) * (TOP_GEAR_T / RING_T); // ~1.3406

    public static final double SERVO_RANGE_DEG = 355; // TODO-measure: servo model not chosen yet
    public static final double SERVO_ZERO_POS = 0.5; // servo position with the turret straight ahead // TODO-measure
    public static final int SERVO_DIRECTION = 1; // 1 or -1, flip if the turret turns the wrong way // TODO-measure
    // Reachable turret travel, centered on straight ahead. Capped at one turn: no wrap logic, targets past it are clamped.
    public static final double MAX_TURRET_DEG = Math.min(360, SERVO_RANGE_DEG * TURRET_DEG_PER_SERVO_DEG) / 2;

    /** Turret pivot in the robot frame (x forward, y right). Also the launcher position SOTM leads from. TODO-measure: DECODE value. */
    public static final Point ROBOT_REL_TURRET_POINT = new Point(-24.51971493, 2.50233587);

    // Driver turret offset added to every aim angle, degrees. TeleOp gamepad 2 dpad left/right changes it.
    public static double TURRET_OFFSET_DEG = 0;

}
