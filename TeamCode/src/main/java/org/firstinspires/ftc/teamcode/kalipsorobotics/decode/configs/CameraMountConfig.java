package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;

@Config
public class CameraMountConfig {

    // Position of the pan/tilt pivot relative to the turret, with both servos centered.
    // TODO measure once the gimbal is built - placeholder copied from the old fixed TURRET_REL_CAM_POS.
    public static Position TURRET_REL_GIMBAL_PIVOT_POS = new Position(-147.565, 0.977, 0);

    // Camera's fixed offset from the pivot point itself (ideally ~0 if the lens sits at the pivot).
    // TODO measure from CAD once the gimbal is built.
    public static Position PIVOT_REL_CAM_POS = new Position(0, 0, 0);

    // Pan (yaw, left/right) servo range. Servo position 0.0 -> PAN_MIN_ANGLE_DEG, 1.0 -> PAN_MAX_ANGLE_DEG.
    public static double PAN_MIN_ANGLE_DEG = -90;
    public static double PAN_MAX_ANGLE_DEG = 90;
    public static boolean PAN_FLIP_DIRECTION = false;

    // Tilt (pitch, up/down) servo range. Exposed for aiming/telemetry; see CameraMount for
    // why it isn't part of the planar (x, y, theta) transform used for localization.
    public static double TILT_MIN_ANGLE_DEG = -45;
    public static double TILT_MAX_ANGLE_DEG = 45;
    public static boolean TILT_FLIP_DIRECTION = false;
}
