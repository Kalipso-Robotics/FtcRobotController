package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

/**
 * Shoot-on-the-move tunables. Units: mm, ms, rad.
 *
 * Tuning recipe: (1) recollect flight times from slow-mo video and refit A/B,
 * (2) measure TurretConfig.ROBOT_REL_TURRET_POINT, (3) tune RELEASE_LATENCY_MS until stationary and strafing
 * shots both land.
 */
@Config
public class SOTMConfig {
    // ---- Switch ----

    /** Dashboard kill switch. At standstill the deadband makes the solution equal the static aim. */
    public static boolean enabled = false;

    // ---- Measured (physical constants, not Dashboard-tuned) ----

    /** Goal points in our alliance's own frame (no mirroring, see FieldConfig). Index 0 = Hive Cell A, 1 = Cell B. */
    public static Point[] GOALS = {FieldConfig.aTargetPoint, FieldConfig.bTargetPoint};

    // Launcher position = TurretConfig.ROBOT_REL_TURRET_POINT (the ball leaves from the turret).

    // ---- Tuned: shot model ----

    public static double RELEASE_LATENCY_MS = 50;

    /** Flight time t = A + B * distance. Fit of the old LUT (RMS 86 ms), recollect later. */
    public static double FLIGHT_TIME_A_MS = 344;
    public static double FLIGHT_TIME_B_MS_PER_MM = 0.186;

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
