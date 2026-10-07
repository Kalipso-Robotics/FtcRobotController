package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.SOTMConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.PoseHistory;
import org.junit.Before;
import org.junit.Test;

public class SOTMTest {
    private static final double GX = 3000, GY = 0;

    @Before
    public void config() {
        SOTMConfig.RELEASE_LATENCY_MS = 0;
        SOTMConfig.LAUNCHER_OFFSET_X_MM = 0;
        SOTMConfig.LAUNCHER_OFFSET_Y_MM = 0;
        SOTMConfig.FLIGHT_TIME_A_MS = 344;
        SOTMConfig.FLIGHT_TIME_B_MS_PER_MM = 0.186;
        SOTMConfig.MAX_ITERATIONS = 6;
        SOTMConfig.CONVERGE_TOL_MM = 1;
    }

    private static PoseHistory.Motion motion(double x, double y, double th, double vx, double vy, double w) {
        return new PoseHistory.Motion(1000, x, y, th, vx, vy, w, 0, 0, 0);
    }

    private static SOTM.Solution solve(PoseHistory.Motion m) {
        return SOTM.solve(GX, GY, 0, m, m.sampleNanos);
    }

    @Test
    public void stationary_equalsStaticAim() {
        SOTM.Solution s = solve(motion(0, 1000, 0.3, 0, 0, 0));
        assertEquals(Math.atan2(GY - 1000, GX), s.aimHeadingRad, 1e-12);
        assertEquals(Math.hypot(GX, 1000), s.shotDistanceMM, 1e-9);
        assertTrue(s.converged);
    }

    @Test
    public void jitterBelowDeadband_isZeroed() {
        SOTM.Solution s = solve(motion(0, 0, 0, 0.01, 0.01, 0.0001));
        assertEquals(0, s.aimHeadingRad, 1e-12);
    }

    @Test
    public void strafing_leadsOppositeToVelocity() {
        double v = 1.0; // mm/ms, +y
        SOTM.Solution s = solve(motion(0, 0, 0, 0, v, 0));
        // Ball inherits +y velocity, so aim must point to -y of the goal.
        assertTrue(s.aimHeadingRad < 0);
        // Converged lead: tan(aim) = -v*t/dist along x
        assertEquals(-v * s.flightTimeMS, Math.tan(s.aimHeadingRad) * GX, 2.0);
        assertTrue(s.converged);
    }

    @Test
    public void drivingTowardGoal_shortensShot() {
        double v = 1.0;
        SOTM.Solution s = solve(motion(0, 0, 0, v, 0, 0));
        assertEquals(GX - v * s.flightTimeMS, s.shotDistanceMM, 2.0);
    }

    @Test
    public void spinWithOffset_addsOmegaCrossR() {
        SOTMConfig.LAUNCHER_OFFSET_X_MM = 100; // r = (100, 0) at heading 0
        double w = 0.002; // rad/ms -> v_L = w * (0, 100) = (0, 0.2) mm/ms in +y
        SOTM.Solution spin = solve(motion(0, 0, 0, 0, 0, w));
        // Same translation without spin must aim differently from the spinning case.
        SOTM.Solution still = solve(motion(0, 0, 0, 0, 0, 0));
        assertTrue(spin.aimHeadingRad < still.aimHeadingRad);
    }

    @Test
    public void unconvergedWhenIterationsExhausted() {
        SOTMConfig.MAX_ITERATIONS = 1;
        SOTMConfig.CONVERGE_TOL_MM = 1e-9;
        assertTrue(!solve(motion(0, 0, 0, 1, 1, 0)).converged);
    }
}
