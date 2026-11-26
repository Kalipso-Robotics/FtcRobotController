package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

import com.acmerobotics.dashboard.config.Config;


@Config
public class TurretConfig {
    //==========TURRET=========
    // One Axon MAX servo, no encoder. Servo 60T -> 20T pinion, same shaft 37T -> 92T turret ring (servo/pinion were 50T/15T until 2026-11-23).
    // Two external meshes, so the turret turns the same direction as the servo.
    public static final double SERVO_GEAR_T = 60;
    public static final double PINION_T = 20;
    public static final double TOP_GEAR_T = 37;
    public static final double RING_T = 92;
    public static final double TURRET_DEG_PER_SERVO_DEG = (SERVO_GEAR_T / PINION_T) * (TOP_GEAR_T / RING_T); // ~1.2065

    // Axon MAX servo mode: 500-2500 us maps onto its programmed range (355 default). Turret sets that PWM range.
    // If the Axon was reprogrammed to a different range, change this to match.
    public static final double SERVO_RANGE_DEG = 355;
    // [2026-11-22] 0.2589 pointed the shooter out the BACK: collector commanded 45 right from the corner, turret went
    // ~225 right. Front = 0.2589 + 180 deg of turret (0.3782 of servo, toward higher pos with dir -1). Fine-tune with
    // TurretServoTest (hold bumper + dpad).
    // [2026-11-23] Everything above was measured on the old 50T/15T gears; re-measure after the 60T/20T swap.
    public static final double SERVO_ZERO_POS = 0.4322; // TODO-measure
    // Positive turret angle = clockwise seen from above (to the robot's right, same as heading). Check with the
    // Test_Shooter "turret" log: a positive turretDeg must swing the turret right; if it swings left, flip this.
    // [2026-11-22] -1: straight ahead needed a HIGHER servo pos than 0.5 to undo "a little right", so higher pos =
    // turret left (servo mounted facing down). Verify: TurretServoTest dpad right (+5 deg) must turn it right.
    public static final int SERVO_DIRECTION = -1;
    // Reachable turret travel: the turret angle at each end of the servo, capped at +-180 (no wrap logic, targets past
    // a limit are clamped). Only symmetric when SERVO_ZERO_POS = 0.5; 0.6371 with dir -1 gives -155.4..+180.
    private static final double SERVO_END_0_DEG = (0 - SERVO_ZERO_POS) * SERVO_RANGE_DEG * TURRET_DEG_PER_SERVO_DEG / SERVO_DIRECTION;
    private static final double SERVO_END_1_DEG = (1 - SERVO_ZERO_POS) * SERVO_RANGE_DEG * TURRET_DEG_PER_SERVO_DEG / SERVO_DIRECTION;
    public static final double MIN_TURRET_DEG = Math.max(-180, Math.min(SERVO_END_0_DEG, SERVO_END_1_DEG)); // furthest left
    public static final double MAX_TURRET_DEG = Math.min(180, Math.max(SERVO_END_0_DEG, SERVO_END_1_DEG));  // furthest right

    /**
     * Turret pivot in the robot frame (x forward, y right, mm), measured from the robot centre (middle of the 4 mecanum
     * wheels), which is the point the OctoQuad localizer reports (OctoConfig.TCP_OFFSET_MM_*). Also the launcher
     * position SOTM leads from.
     * From CAD (BioBuzz Robot Assembly V1, lazy susan bearing center), 2026-11-23: 5.6 mm AHEAD of centre, centred
     * side to side. Forward = the way the Arducam looks (+y in CAD); the 11-22 value (-5.8) had forward flipped.
     * (Was (-24.52, 2.50), the DECODE robot.)
     */
    public static final Point ROBOT_REL_TURRET_POINT = new Point(5.6, 0.0);

    // Driver turret offset added to every aim angle, degrees. TeleOp gamepad 2 dpad left/right changes it.
    public static double TURRET_OFFSET_DEG = 0;

}
