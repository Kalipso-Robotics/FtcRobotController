package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.SOTMConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.PoseHistory;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * Shoot-on-the-move. Predicts the pose at ball release, then iterates a virtual goal
 * G' = G - v_launcher * flightTime(|G' - L|) so aiming at G' lands the ball on G.
 * Runs on its own thread (OpModeUtilities.runSOTMExecutorService) and publishes a Solution
 * to SharedData for the turret and shooter to read.
 */
public class SOTM {

    public static class Solution {
        public final int goalIndex;
        public final double aimHeadingRad;      // field heading from launcher to virtual goal
        public final double releaseHeadingRad;  // predicted robot heading at release
        public final double shotDistanceMM;     // launcher to virtual goal, for the RPS lookup
        public final double flightTimeMS;
        public final boolean converged;
        public final long sampleNanos;

        public Solution(int goalIndex, double aimHeadingRad, double releaseHeadingRad,
                        double shotDistanceMM, double flightTimeMS, boolean converged, long sampleNanos) {
            this.goalIndex = goalIndex;
            this.aimHeadingRad = aimHeadingRad;
            this.releaseHeadingRad = releaseHeadingRad;
            this.shotDistanceMM = shotDistanceMM;
            this.flightTimeMS = flightTimeMS;
            this.converged = converged;
            this.sampleNanos = sampleNanos;
        }

        public boolean isUsable() {
            return SOTMConfig.enabled && converged
                    && (System.nanoTime() - sampleNanos) / 1e6 <= SOTMConfig.MAX_SOLUTION_AGE_MS;
        }

        /** Turret angle relative to the robot at release (before any ticks offset). */
        public double turretAngleRad() {
            return MathFunctions.angleWrapRad(aimHeadingRad - releaseHeadingRad);
        }
    }

    public static boolean isActive() {
        return usableSolution() != null;
    }

    /** The published solution if it is enabled, fresh and converged, else null. */
    public static Solution usableSolution() {
        if (!SOTMConfig.enabled) return null;
        Solution s = SharedData.getSOTMSolution();
        return s != null && s.isUsable() ? s : null;
    }

    private final OpModeUtilities opModeUtilities;
    private volatile long lastSampleNanos = 0;
    private volatile boolean running = false;

    public boolean isRunning() {
        return running;
    }

    public void setRunning(boolean running) {
        this.running = running;
    }

    public SOTM(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }

    public long getLastSampleNanos() {
        return lastSampleNanos;
    }

    /** One tick: fit the latest motion, solve for the active goal, publish. */
    public void update() {
        PoseHistory.Motion motion = SharedData.getOdometryWheelIMUMotion((long) SOTMConfig.MOTION_FIT_WINDOW_MS);
        if (motion == null) return;
        lastSampleNanos = motion.sampleNanos;
        int goalIndex = SharedData.getSOTMActiveGoal();
        Point goal = goal(goalIndex);
        Solution s = solve(goal.getX(), goal.getY(), goalIndex, motion, System.nanoTime());
        SharedData.setSOTMSolution(s);
        KLog.d("SOTM", () -> String.format(
                "goal %d aim %.1fdeg release %.1fdeg dist %.0f t %.0fms converged %b v(%.2f,%.2f) w %.4f",
                s.goalIndex, Math.toDegrees(s.aimHeadingRad), Math.toDegrees(s.releaseHeadingRad),
                s.shotDistanceMM, s.flightTimeMS, s.converged, motion.vx, motion.vy, motion.omega));
    }

    /** Field position of the goal, mirrored by the current alliance. */
    static Point goal(int index) {
        Point p = SOTMConfig.GOALS[index < SOTMConfig.GOALS.length ? index : 0];
        return p.multiplyY(SharedData.getAllianceColor().getPolarity());
    }

    private static double flightTimeMS(double distanceMM) {
        return SOTMConfig.FLIGHT_TIME_A_MS + SOTMConfig.FLIGHT_TIME_B_MS_PER_MM * distanceMM;
    }

    /** p + v*t + a*t^2/2 */
    private static double extrapolate(double p, double v, double a, double t) {
        return p + v * t + 0.5 * a * t * t;
    }

    /** Pure. Goal in field mm; motion in mm, rad, ms. */
    public static Solution solve(double gx, double gy, int goalIndex, PoseHistory.Motion m, long nowNanos) {
        double vx = m.vx, vy = m.vy, w = m.omega, ax = m.ax, ay = m.ay, al = m.alpha;
        double tau = SOTMConfig.RELEASE_LATENCY_MS + (nowNanos - m.sampleNanos) / 1e6;
        if (vx * vx + vy * vy < SOTMConfig.MIN_SPEED_MM_PER_MS * SOTMConfig.MIN_SPEED_MM_PER_MS && Math.abs(w) < SOTMConfig.MIN_OMEGA_RAD_PER_MS) {
            vx = vy = w = ax = ay = al = 0;
            tau = 0;
        }

        // Release state
        double px = extrapolate(m.x, vx, ax, tau);
        double py = extrapolate(m.y, vy, ay, tau);
        double th = extrapolate(m.theta, w, al, tau);
        double vrx = vx + ax * tau, vry = vy + ay * tau, wr = w + al * tau;

        // Launcher position and velocity (omega x r)
        double c = Math.cos(th), s = Math.sin(th);
        double rx = c * SOTMConfig.LAUNCHER_OFFSET_X_MM - s * SOTMConfig.LAUNCHER_OFFSET_Y_MM;
        double ry = s * SOTMConfig.LAUNCHER_OFFSET_X_MM + c * SOTMConfig.LAUNCHER_OFFSET_Y_MM;
        double lx = px + rx, ly = py + ry;
        double vlx = vrx - wr * ry, vly = vry + wr * rx;

        // sqrt instead of Math.hypot (hypot is ~10x slower and we don't need its overflow safety)
        double gvx = gx, gvy = gy;
        double d = Math.sqrt((gvx - lx) * (gvx - lx) + (gvy - ly) * (gvy - ly));
        double t = flightTimeMS(d);
        double tolSq = SOTMConfig.CONVERGE_TOL_MM * SOTMConfig.CONVERGE_TOL_MM;
        boolean converged = false;
        for (int i = 0; i < SOTMConfig.MAX_ITERATIONS && !converged; i++) {
            double nx = gx - vlx * t, ny = gy - vly * t;
            converged = (nx - gvx) * (nx - gvx) + (ny - gvy) * (ny - gvy) < tolSq;
            gvx = nx;
            gvy = ny;
            d = Math.sqrt((gvx - lx) * (gvx - lx) + (gvy - ly) * (gvy - ly));
            t = flightTimeMS(d);
        }
        return new Solution(goalIndex, Math.atan2(gvy - ly, gvx - lx), th, d, t, converged, m.sampleNanos);
    }
}
