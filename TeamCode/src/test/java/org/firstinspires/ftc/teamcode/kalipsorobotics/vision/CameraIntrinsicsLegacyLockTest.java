package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Vector3d;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

/**
 * Characterisation test: freezes the legacy bottom-edge floor ray and size ranging so the
 * ball-localisation rewrite cannot silently change what existing callers get. Golden values
 * come from an exact replica of the original maths (cross-checked against a logged CSV row).
 * Delete this once the callers have been migrated to estimateBall.
 */
public class CameraIntrinsicsLegacyLockTest {

    private static final double TOL = 1e-4;
    private static final CameraIntrinsics CI = CameraIntrinsics.ARDUCAM; // pitch 29 deg
    private static final Position POSE = new Position(1000.0, -500.0, Math.toRadians(30));

    // {u, v, robotX, robotY, fieldX, fieldY, dist}
    private static final double[][] GOLDEN = {
            {350, 400, -157.535528, 373.063226, 1401.849995, -449.898157, 404.961249},
            {108, 268, 65.828548, 501.258842, 1401.188616, -192.361384, 505.562878},
            {600, 300, -368.976743, 462.058458, 1584.642734, -588.514003, 591.305213},
            {320, 240, -127.436926, 541.938551, 1533.051016, -339.394340, 556.720364},
            {320, 100, -104.351346, 932.467198, 1859.715955, -124.137318, 938.287951},
    };

    @Test
    public void floorRay_robotFieldAndDistance() {
        for (double[] g : GOLDEN) {
            String at = " at pixel " + g[0] + "," + g[1];
            Point robot = CI.calculateRobotFramePos(g[0], g[1]);
            assertEquals("robot x" + at, g[2], robot.getX(), TOL);
            assertEquals("robot y" + at, g[3], robot.getY(), TOL);

            Point field = CI.calculateWorldPos(g[0], g[1], POSE);
            assertEquals("field x" + at, g[4], field.getX(), TOL);
            assertEquals("field y" + at, g[5], field.getY(), TOL);

            assertEquals("dist" + at, g[6], CI.getDistanceFromRobot(g[0], g[1], POSE), TOL);
        }
    }

    @Test
    public void floorRay_recognitionOverloadUsesBottomMiddle() {
        VisionRecognition r = new VisionRecognition("t", 1f, 300, 250, 380, 330);
        Point direct = CI.calculateRobotFramePos(340, 330);
        assertEquals(-149.690588, direct.getX(), TOL);
        assertEquals(430.830379, direct.getY(), TOL);
        assertEquals(CI.getDistanceFromRobot(340, 330, POSE), CI.getDistanceFromRobot(r, POSE), TOL);
    }

    @Test
    public void sizeRanging_unchanged() {
        VisionRecognition r = new VisionRecognition("t", 1f, 300, 250, 380, 330);
        Point p = CI.calculateRobotFramePosFromSize(r, 127);
        assertEquals(-139.571467, p.getX(), TOL);
        assertEquals(804.183381, p.getY(), TOL);
    }

    @Test
    public void aboveHorizon_nullAndInfinite() {
        // At 29 deg pitch the horizon is at v ~ -82, above the image top, so use v = -200.
        assertNull(CI.calculateRobotFramePos(320, -200));
        assertTrue(Double.isInfinite(CI.getDistanceFromRobot(320, -200, POSE)));
    }

    @Test
    public void withMount_matchesOriginalMath() {
        CameraIntrinsics ci = CI.withMount(Math.toRadians(24), new Vector3d(-157.548, 236.163, 163.470));
        Point p = ci.calculateRobotFramePos(108, 268);
        assertEquals(98.754727, p.getX(), TOL);
        assertEquals(573.191712, p.getY(), TOL);
    }
}
