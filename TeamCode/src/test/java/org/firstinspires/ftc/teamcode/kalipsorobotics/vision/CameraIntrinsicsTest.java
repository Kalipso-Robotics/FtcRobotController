package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Matrix;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Vector3d;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblobbing.DetectedBlob;
import org.junit.After;
import org.junit.Test;
import org.opencv.core.Rect;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

/**
 * Tests for the edge-angle ball localisation (CameraIntrinsics.estimateBall): per-axis
 * averaged edge angles, ray intersected with the plane at the ball's radius, plus the
 * angular-size depth cross-check.
 */
public class CameraIntrinsicsTest {

    private static final double R_NECTAR = 35.6;
    private static final Vector3d LENS = new Vector3d(0, 300, 0);

    /** f=500, principal point (320,240), no distortion -- only the numbers these tests need. */
    private static final CameraIntrinsics CI = new CameraIntrinsics(500, 500, 320, 240, 0, LENS);

    @After
    public void restoreDistortionFlag() {
        CameraIntrinsics.APPLY_DISTORTION = false;
    }

    private static CameraPose level() {
        return CameraPose.fromAngles(0, 0, 0, LENS);
    }

    @Test
    public void levelCamera_forwardFromVerticalEdges() {
        BallEstimate e = CI.estimateBall(300, 340, 382.12, 429.04, R_NECTAR, level());
        assertEquals(800.0, e.robotPos.getY(), 0.1);
        assertEquals(0.0, e.robotPos.getX(), 0.1);
    }

    @Test
    public void levelCamera_sidewaysTowardUGreaterThanCx() {
        BallEstimate e = CI.estimateBall(360.18, 405.06, 382.12, 429.04, R_NECTAR, level());
        assertEquals(800.0, e.robotPos.getY(), 0.1);
        // +x is LEFT in the robot frame, so u > cx is negative x.
        assertEquals(-100.0, e.robotPos.getX(), 0.1);
    }

    @Test
    public void levelCamera_sizeDepthsAgreeWithRay() {
        BallEstimate e = CI.estimateBall(360.18, 405.06, 382.12, 429.04, R_NECTAR, level());
        assertEquals(800.0, e.rayDepthMM, 0.1);
        // Level camera: the robot-frame ray is the camera ray flipped, so z = 1 and y = -tan(theta).
        assertEquals(1.0, e.rayRobot.getZ(), 1e-12);
        assertEquals(-Math.tan(e.thetaRad), e.rayRobot.getY(), 1e-12);
        assertEquals(-Math.tan(e.psiRad), e.rayRobot.getX(), 1e-12);
        // psi/theta are the mean of the edge angles.
        double expPsi = (Math.atan((360.18 - 320) / 500.0) + Math.atan((405.06 - 320) / 500.0)) / 2;
        assertEquals(expPsi, e.psiRad, 1e-12);
        assertEquals(800.0, e.sizeDepthHMM, 0.5);
        assertEquals(800.0, e.sizeDepthVMM, 0.5);
        assertTrue(e.isConsistent());

        // r / sin(delta) is the range inside each axis's plane, not the 3D range:
        // sqrt(800^2 + 100^2) = 806.2 sideways, sqrt(800^2 + 264.4^2) = 842.6 vertically.
        double aL = Math.atan((360.18 - 320) / 500), aR = Math.atan((405.06 - 320) / 500);
        assertEquals(806.2, R_NECTAR / Math.sin((aR - aL) / 2), 0.5);
        double aT = Math.atan((382.12 - 240) / 500), aB = Math.atan((429.04 - 240) / 500);
        assertEquals(842.6, R_NECTAR / Math.sin((aB - aT) / 2), 0.5);
    }

    @Test
    public void averagingPixelsInsteadOfAnglesIsBiased() {
        // Documents why angles are averaged: the centre-pixel ray gives 798.4, not 800.
        double vc = (382.12 + 429.04) / 2;
        double forward = (300 - R_NECTAR) / ((vc - 240) / 500);
        assertEquals(798.4, forward, 0.05);
    }

    @Test
    public void pitchedCentreRay() {
        // Zero-width edges collapse to the single centre pixel (370,290): pitch 30 deg down.
        CameraPose cam = CameraPose.fromAngles(Math.toRadians(30), 0, 0, LENS);
        BallEstimate e = CI.estimateBall(370, 370, 290, 290, R_NECTAR, cam);
        assertEquals(367.8, e.robotPos.getY(), 0.05);
        assertEquals(-45.07, e.robotPos.getX(), 0.01);
        // delta = 0 makes the size depths infinite, so the check is false by design here.
        assertFalse(e.isConsistent());
    }

    @Test
    public void yawRotatesRayOntoLateralAxis() {
        // (cx,cy) pitched 30 deg then yawed +90 deg lands on the lateral axis.
        CameraPose cam = CameraPose.fromAngles(Math.toRadians(30), Math.toRadians(90), 0, LENS);
        BallEstimate e = CI.estimateBall(320, 320, 240, 240, 0.001, cam);
        assertEquals(519.6, e.robotPos.getX(), 0.1);
        assertEquals(0.0, e.robotPos.getY(), 1e-3);
    }

    @Test
    public void cvToRobotIsRotationNotMirror() {
        Matrix m = CameraPose.CV_TO_ROBOT;
        double det = m.get(0, 0) * (m.get(1, 1) * m.get(2, 2) - m.get(1, 2) * m.get(2, 1))
                - m.get(0, 1) * (m.get(1, 0) * m.get(2, 2) - m.get(1, 2) * m.get(2, 0))
                + m.get(0, 2) * (m.get(1, 0) * m.get(2, 1) - m.get(1, 1) * m.get(2, 0));
        assertEquals(1.0, det, 1e-12);
    }

    @Test
    public void beyondFieldRange_isNull() {
        // Level camera 300mm up, ball r=35.6: vB row for a ball ~6000mm out is ~1.5px below cy.
        double v = 240 + 500.0 * (300 - R_NECTAR) / 6000.0;
        assertNull(CI.estimateBall(318, 322, v - 0.3, v + 0.3, R_NECTAR, level()));
    }

    @Test
    public void flatOrUpwardRay_isNull() {
        assertNull(CI.estimateBall(300, 340, 200, 200, R_NECTAR, level()));
    }

    @Test
    public void borderClippedBbox_isNull() {
        DetectedBlob clipped = new DetectedBlob(new Rect(0, 300, 60, 60), 2000, 0.9, "x", 71.2);
        assertNull(CI.estimateBall(clipped, level()));
        DetectedBlob ok = new DetectedBlob(new Rect(300, 300, 60, 60), 2000, 0.9, "x", 71.2);
        assertNotNull(CI.estimateBall(ok, CameraPose.fromAngles(Math.toRadians(30), 0, 0, LENS)));
    }

    @Test
    public void unknownDiameter_isNull() {
        DetectedBlob none = new DetectedBlob(new Rect(300, 300, 60, 60), 2000, 0.9, "Yellow");
        assertNull(CI.estimateBall(none, CameraPose.fromAngles(Math.toRadians(30), 0, 0, LENS)));
    }

    @Test
    public void wrongRadius_isFlaggedInconsistent() {
        BallEstimate e = CI.estimateBall(360.18, 405.06, 382.12, 429.04, 2 * R_NECTAR, level());
        assertNotNull(e);
        assertFalse(e.isConsistent());
    }

    @Test
    public void undistortInvertsDistort_andFlagOffIsPlainPinhole() {
        CameraIntrinsics ci = CameraIntrinsics.ARDUCAM;
        Random rnd = new Random(7);
        for (int i = 0; i < 20; i++) {
            double x = (rnd.nextDouble() - 0.5) * 1.6, y = (rnd.nextDouble() - 0.5) * 1.6;
            double[] d = ci.distort(x, y);
            double[] back = ci.undistort(ci.getCx() + ci.getFx() * d[0], ci.getCy() + ci.getFy() * d[1]);
            assertEquals(x, back[0], 1e-6);
            assertEquals(y, back[1], 1e-6);
        }
        // Flag off (default): estimateBall must equal the pinhole result exactly.
        BallEstimate off = CI.estimateBall(360.18, 405.06, 382.12, 429.04, R_NECTAR, level());
        assertEquals(800.0, off.robotPos.getY(), 0.1);
    }

    @Test
    public void roundTrip_randomPoseBothBallSizes() {
        Random rnd = new Random(42);
        double f = 500, cx = 320, cy = 240;
        int checked = 0;
        for (int i = 0; i < 500; i++) {
            double pitch = Math.toRadians(rnd.nextDouble() * 45);
            double yaw = Math.toRadians((rnd.nextDouble() - 0.5) * 80);
            double roll = Math.toRadians((rnd.nextDouble() - 0.5) * 30);
            Vector3d pos = new Vector3d((rnd.nextDouble() - 0.5) * 400,
                    150 + rnd.nextDouble() * 250, (rnd.nextDouble() - 0.5) * 400);
            double r = rnd.nextBoolean() ? 35.6 : 63.5;
            CameraPose cam = CameraPose.fromAngles(pitch, yaw, roll, pos);

            // Ball centre on the plane y = r, ahead of the camera along its yawed heading.
            double ahead = 300 + rnd.nextDouble() * 1700, side = (rnd.nextDouble() - 0.5) * 800;
            double cyaw = Math.cos(yaw), syaw = Math.sin(yaw);
            double bx = pos.getX() + ahead * -syaw + side * cyaw;   // +x = left; yaw turns left
            double bz = pos.getZ() + ahead * cyaw + side * syaw;
            Vector3d centre = new Vector3d(bx, r, bz);

            // Camera-frame centre = R^T (centre - camPos).
            Vector3d d = new Vector3d(centre.getX() - pos.getX(), centre.getY() - pos.getY(),
                    centre.getZ() - pos.getZ());
            double[] c = new double[3];
            for (int k = 0; k < 3; k++) {
                c[k] = cam.camToRobot.get(0, k) * d.getX() + cam.camToRobot.get(1, k) * d.getY()
                        + cam.camToRobot.get(2, k) * d.getZ();
            }
            if (c[2] < 3 * r) continue;

            double psi0 = Math.atan2(c[0], c[2]), dx = Math.asin(r / Math.hypot(c[0], c[2]));
            double th0 = Math.atan2(c[1], c[2]), dy = Math.asin(r / Math.hypot(c[1], c[2]));
            double uL = cx + f * Math.tan(psi0 - dx), uR = cx + f * Math.tan(psi0 + dx);
            double vT = cy + f * Math.tan(th0 - dy), vB = cy + f * Math.tan(th0 + dy);
            if (uL < 0 || uR > 640 || vT < 0 || vB > 480 || uL > uR || vT > vB) continue;

            BallEstimate e = CI.estimateBall(uL, uR, vT, vB, r, cam);
            assertNotNull("draw " + i, e);
            assertEquals("x, draw " + i, bx, e.robotPos.getX(), 1e-6);
            assertEquals("z, draw " + i, bz, e.robotPos.getY(), 1e-6);
            assertEquals("ray depth, draw " + i, c[2], e.rayDepthMM, 1e-6);
            assertEquals("size V depth, draw " + i, c[2], e.sizeDepthVMM, 1e-6);
            assertEquals("size H depth, draw " + i, c[2], e.sizeDepthHMM, 1e-6);
            checked++;
        }
        assertTrue("too few usable draws: " + checked, checked >= 50);
    }
}
