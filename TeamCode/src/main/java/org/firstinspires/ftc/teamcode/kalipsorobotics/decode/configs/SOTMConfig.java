package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

/**
 * Shoot-on-the-move tunables. Units: mm, ms, rad.
 *
 * Tuning recipe: (1) time one near and one far shot on slow-mo video, fit flight time t = A + B*distance
 * (miss ~= robot speed * flight-time error, so ~50 ms is plenty; no LUT needed),
 * (2) measure TurretConfig.ROBOT_REL_TURRET_POINT, (3) tune RELEASE_LATENCY_MS until stationary and strafing
 * shots both land.
 */
@Config
public class SOTMConfig {
    // ---- Switch ----

    /** Dashboard kill switch: runs the SOTM thread and aims the turret at the virtual goal (and keeps it tracking
     *  during a shot). At standstill the deadband makes the solution equal the static aim. */
    public static boolean aimTurret = true;
    /** Shooter RPS looks up the virtual-goal distance instead of the real one. Only applies while aimTurret is on.
     *  Turn on once the RPS LUT and PIDF are tuned. */
    public static boolean adjustShooterRPS = false;

    // ---- Measured (physical constants, not Dashboard-tuned) ----

    /** Goal points in our alliance's own frame (no mirroring, see FieldConfig). Index 0 = Hive Cell A, 1 = Cell B (same points as SharedData.getTargetPoint()). */
    public static Point[] GOALS = {ShooterConfig.TARGET_POINT_1, ShooterConfig.TARGET_POINT_2};

    // Launcher position = TurretConfig.ROBOT_REL_TURRET_POINT (the ball leaves from the turret).

    // ---- Tuned: shot model ----

    public static double RELEASE_LATENCY_MS = 50;

    /** Flight time t = A + B * distance. Rough constant ~1 s for now; refit from slow-mo (old LUT fit was 344 + 0.186*d). */
    public static double FLIGHT_TIME_A_MS = 1000;
    public static double FLIGHT_TIME_B_MS_PER_MM = 0;

    // ---- Tuned: motion estimate ----

    public static double MOTION_FIT_WINDOW_MS = 60;
    /** Below both of these the solution is the static aim. */
    public static double MIN_SPEED_MM_PER_MS = 0.05;
    public static double MIN_OMEGA_RAD_PER_MS = 0.0005;

    // ---- Solver limits ----

    public static int MAX_ITERATIONS = 6;
    public static double CONVERGE_TOL_MM = 1;
    public static double MAX_SOLUTION_AGE_MS = 50;
}
