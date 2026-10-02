package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Matrix;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Vector3d;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblobbing.DetectedBlob;
import org.junit.Test;
import org.opencv.core.Rect;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

/**
 * Tests for the ball-ray localisation added to CameraIntrinsics: a centre-pixel ray through a
 * full cameraToRobot rotation, intersected with the horizontal plane at the ball's own radius
 * instead of the floor.
 */
public class CameraIntrinsicsTest {

    private static final double TOL = 1e-6;

    /** f=500, principal point (320,240), no distortion -- only the numbers this test needs. */
    private static CameraIntrinsics intrinsics(double mountAngleForOldPath, Vector3d cameraOffset) {
        return new CameraIntrinsics(500, 500, 320, 240, mountAngleForOldPath, cameraOffset);
    }

    @Test
    public void handDerivedCase_ballRadiusPlane() {
        // f=500, principal point (320,240), pitch 30 deg down, lens 300mm above the floor,
        // pixel (370,290), r=35.6mm -> forward 367.8mm, lateral -45.07mm (u>cx maps to -x,
        // since +x is LEFT in this frame).
        CameraIntrinsics ci = intrinsics(0, new Vector3d(0, 300, 0));
        Matrix camToRobot = CameraIntrinsics.cameraToRobot(Math.toRadians(30), 0, 0);

        Point result = ci.calculateBallRobotFramePos(370, 290, 35.6, camToRobot, new Vector3d(0, 300, 0));

        assertEquals(-45.07, result.getX(), 0.01);
        assertEquals(367.8, result.getY(), 0.05);
    }

    @Test
    public void pitchOnlyRotation_matchesOldFloorRayPitchBehavior() {
        // With radius = 0 (floor plane) and yaw = roll = 0, the new ray-plane core must
        // reproduce calculateRobotFramePos's pitch-only transform exactly -- that is the
        // pitch behaviour this rewrite is required to keep.
        double pitchDeg = 29.0;
        Vector3d camOffset = new Vector3d(-157.548, 236.163, 163.470);
        CameraIntrinsics ci = intrinsics(Math.toRadians(pitchDeg), camOffset);
        Matrix camToRobot = CameraIntrinsics.cameraToRobot(Math.toRadians(pitchDeg), 0, 0);

        double[][] pixels = {
                {320, 400}, {200, 350}, {450, 420}, {320, 300}
        };
        for (double[] px : pixels) {
            Point oldResult = ci.calculateRobotFramePos(px[0], px[1]);
            Point newResult = ci.calculateBallRobotFramePos(px[0], px[1], 0, camToRobot, camOffset);
            assertEquals("x mismatch at pixel " + px[0] + "," + px[1],
                    oldResult.getX(), newResult.getX(), 1e-6);
            assertEquals("y mismatch at pixel " + px[0] + "," + px[1],
                    oldResult.getY(), newResult.getY(), 1e-6);
        }
    }

    @Test
    public void yawRotatesRayOntoLateralAxis() {
        // A straight-ahead pixel (u=cx, v=cy), pitched 30deg down (so the ray still has a
        // downward component to intersect the floor) then yawed +90deg, should land almost
        // entirely on the lateral (x) axis instead of forward (z/y).
        CameraIntrinsics ci = intrinsics(0, new Vector3d(0, 300, 0));
        Matrix camToRobot = CameraIntrinsics.cameraToRobot(Math.toRadians(30), Math.toRadians(90), 0);

        Point result = ci.calculateBallRobotFramePos(320, 240, 0, camToRobot, new Vector3d(0, 300, 0));

        assertEquals(519.615, result.getX(), 0.01);
        assertEquals(0, result.getY(), 1e-6);
    }

    @Test
    public void detectionOverload_returnsNullWithoutKnownDiameter() {
        CameraIntrinsics ci = intrinsics(0, new Vector3d(0, 300, 0));
        Matrix camToRobot = CameraIntrinsics.cameraToRobot(Math.toRadians(30), 0, 0);

        DetectedBlob noDiameter = new DetectedBlob(new Rect(345, 265, 50, 50), 2000, 0.9, "Yellow");
        assertEquals(0.0, noDiameter.getRadiusMM(), TOL);
        assertNull(ci.calculateBallRobotFramePos(noDiameter, camToRobot, new Vector3d(0, 300, 0)));

        DetectedBlob withDiameter = new DetectedBlob(new Rect(345, 265, 50, 50), 2000, 0.9, "Yellow", 71.2);
        assertEquals(35.6, withDiameter.getRadiusMM(), TOL);
        Point result = ci.calculateBallRobotFramePos(withDiameter, camToRobot, new Vector3d(0, 300, 0));
        assertTrue("known-diameter detection should resolve to a point", result != null);
    }
}
