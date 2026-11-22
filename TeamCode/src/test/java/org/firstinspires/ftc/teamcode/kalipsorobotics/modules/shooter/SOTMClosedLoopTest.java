package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter;

import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.SOTMConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.PoseHistory;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.junit.Before;
import org.junit.Test;

/**
 * SOTM end to end: odometry samples of a robot driving, turning and accelerating -> PoseHistory fit -> SOTM.solve ->
 * turret angle (TurretAutoAlign) -> ball flown from the TRUE robot state at release. The ball must land on the Cell.
 * Ball model = the one the shooter is tuned for: launched toward the turret heading, covering the commanded distance
 * in flightTime(distance), plus the launcher's own velocity (robot velocity + omega x launcher offset).
 */
public class SOTMClosedLoopTest {

    private static final long MS = 1_000_000L;
    private static final long LAST_SAMPLE = 10_000 * MS;
    private static final long PROCESSING_DELAY = 15 * MS; // solve runs a bit after the newest odometry sample

    @Before
    public void config() {
        SOTMConfig.enabled = true;
        SOTMConfig.RELEASE_LATENCY_MS = 50;
        SOTMConfig.FLIGHT_TIME_A_MS = 344;
        SOTMConfig.FLIGHT_TIME_B_MS_PER_MM = 0.186;
        SOTMConfig.MAX_ITERATIONS = 6;
        SOTMConfig.CONVERGE_TOL_MM = 1;
    }

    /** Truth: quadratic in time (constant acceleration and angular acceleration). t in ms, 0 at the newest sample. */
    private static final class Drive {
        final double x0, y0, th0, vx, vy, w, ax, ay, al;

        Drive(double x0, double y0, double th0, double vx, double vy, double w, double ax, double ay, double al) {
            this.x0 = x0; this.y0 = y0; this.th0 = th0; this.vx = vx; this.vy = vy; this.w = w; this.ax = ax; this.ay = ay; this.al = al;
        }

        double x(double t) { return x0 + vx * t + 0.5 * ax * t * t; }
        double y(double t) { return y0 + vy * t + 0.5 * ay * t * t; }
        double th(double t) { return th0 + w * t + 0.5 * al * t * t; }
    }

    /** Miss distance in mm at the Cell, with SOTM on or with the plain static aim. */
    private static double miss(Drive d, Point goal, boolean sotm) {
        PoseHistory history = new PoseHistory(64);
        for (int k = 20; k >= 0; k--) { // 5 ms odometry, 100 ms of history
            double t = -5.0 * k;
            history.record(LAST_SAMPLE + (long) (t * MS), new Position(d.x(t), d.y(t), d.th(t)));
        }
        PoseHistory.Motion m = history.fit((long) (SOTMConfig.MOTION_FIT_WINDOW_MS * MS));
        long now = LAST_SAMPLE + PROCESSING_DELAY;
        SOTM.Solution s = SOTM.solve(goal.getX(), goal.getY(), 0, m, now);

        double turretRad, shotDistance;
        if (sotm) {
            turretRad = TurretAutoAlign.calculateTargetAngleRad(s.aimHeadingRad, s.releaseHeadingRad, 0);
            shotDistance = s.shotDistanceMM;
        } else {
            Position here = new Position(m.x, m.y, m.theta);
            turretRad = TurretAutoAlign.calculateTargetAngleRad(goal, here, 0);
            shotDistance = here.toPoint().distanceTo(goal);
        }

        // True state at release.
        double tr = SOTMConfig.RELEASE_LATENCY_MS + PROCESSING_DELAY / 1e6;
        double th = d.th(tr), w = d.w + d.al * tr;
        double ox = TurretConfig.ROBOT_REL_TURRET_POINT.getX(), oy = TurretConfig.ROBOT_REL_TURRET_POINT.getY();
        double rx = Math.cos(th) * ox - Math.sin(th) * oy, ry = Math.sin(th) * ox + Math.cos(th) * oy;
        double lx = d.x(tr) + rx, ly = d.y(tr) + ry;
        double vlx = d.vx + d.ax * tr - w * ry, vly = d.vy + d.ay * tr + w * rx;

        double dir = th + turretRad;
        double flight = SOTMConfig.FLIGHT_TIME_A_MS + SOTMConfig.FLIGHT_TIME_B_MS_PER_MM * shotDistance;
        double bx = lx + Math.cos(dir) * shotDistance + vlx * flight;
        double by = ly + Math.sin(dir) * shotDistance + vly * flight;
        return Math.hypot(bx - goal.getX(), by - goal.getY());
    }

    @Test
    public void strafingPastTheHive_landsOnTheCell() {
        // 1.2 m/s across the field, slowing down, robot turning; ~1.5 m from Cell A.
        Drive d = new Drive(900, 3300, Math.toRadians(-60), 1.2, 0.2, 0.0015, -0.002, 0.0005, 0.00001);
        double withSotm = miss(d, FieldConfig.aTargetPoint, true);
        double without = miss(d, FieldConfig.aTargetPoint, false);
        assertTrue("SOTM miss " + withSotm + " mm", withSotm < 10);
        assertTrue("static aim should miss badly, missed " + without + " mm", without > 300);
    }

    @Test
    public void drivingAwayAndSpinning_landsOnTheCell() {
        Drive d = new Drive(1500, 600, Math.toRadians(80), -0.1, -0.8, -0.003, 0, 0.001, 0);
        assertTrue(miss(d, FieldConfig.bTargetPoint, true) < 10);
    }

    @Test
    public void standingStill_isTheStaticAim() {
        Drive d = new Drive(1200, 3000, 0, 0, 0, 0, 0, 0, 0);
        assertTrue(miss(d, FieldConfig.aTargetPoint, true) < 1e-6);
        // The static aim points from the robot center, not the turret pivot: off by the pivot's few-mm side offset.
        assertTrue(miss(d, FieldConfig.aTargetPoint, false) < 30);
    }
}
