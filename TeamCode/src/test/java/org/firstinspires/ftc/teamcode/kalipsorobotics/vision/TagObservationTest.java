package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.TagObservation;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * Pins TagObservation.fromOpenCvPose() to the same output convention the Limelight path
 * already produces, because AprilTagDetectionAction's transform chain and its spike
 * thresholds were tuned against that convention. If a webcam-backed camera produced a
 * different sign or a 180-degree-rotated heading, relocalization would silently place the
 * robot in the wrong half of the field instead of failing loudly.
 *
 * THE CONVENTION UNDER TEST (see fromOpenCvPose javadoc):
 *   input  camera frame: +X right, +Y down, +Z forward
 *   input  tag frame:    +X right, +Y down, +Z through the tag away from the viewer
 *   output camRelTagPos: x = mm in FRONT of the tag face
 *                        y = mm along the tag's own right
 *                        theta = camera optical-axis heading, 180 deg when staring at the tag
 *
 * Each case below is stated as "where the camera physically is", then converted by hand
 * into the solver's tag-in-camera pose, so the test fails if the transpose or a sign flips.
 */
public class TagObservationTest {

    private static final double MM_TOLERANCE  = 1e-6;
    private static final double DEG_TOLERANCE = 1e-6;

    /** Row-major 3x3, as the SDK's rawPose.R is. */
    private static MatrixF rotation(double... rowMajor) {
        float[] values = new float[9];
        for (int i = 0; i < 9; i++) {
            values[i] = (float) rowMajor[i];
        }
        return new GeneralMatrixF(3, 3, values);
    }

    private static MatrixF identity() {
        return rotation(1, 0, 0,
                        0, 1, 0,
                        0, 0, 1);
    }

    /** Right-handed rotation about the +Y (down) axis. */
    private static MatrixF rotationAboutY(double radians) {
        double c = Math.cos(radians);
        double s = Math.sin(radians);
        return rotation( c, 0, s,
                         0, 1, 0,
                        -s, 0, c);
    }

    // -------------------------------------------------------------------------

    /**
     * Camera parked 1000mm dead in front of the tag, staring straight at it.
     *
     * The tag is then 1000mm straight down the optical axis, and the tag's axes line up
     * with the camera's, so R is the identity. This is the anchor case: the Limelight
     * path yields theta = 180 deg here (its "180 + pitch" with pitch = 0), and so must this.
     */
    @Test
    public void cameraStaringStraightAtTag() {
        TagObservation observation = TagObservation.fromOpenCvPose(
                24, 0, 0, 1000, identity());

        Position camRelTag = observation.getCamRelTagPos();
        assertEquals(1000, camRelTag.getX(), MM_TOLERANCE);
        assertEquals(0, camRelTag.getY(), MM_TOLERANCE);
        assertEquals(180, Math.abs(Math.toDegrees(camRelTag.getTheta())), DEG_TOLERANCE);
        assertEquals(0, observation.getRawPitchDeg(), DEG_TOLERANCE);
        assertEquals(24, observation.getTagId());
    }

    /**
     * Same distance, but the camera has slid 200mm to the viewer's right of the tag while
     * still pointing straight ahead (parallel to the tag's normal, not aimed at it).
     *
     * Two things must come out of this. The tag has to appear on the LEFT of the image
     * (tag-in-camera x is negative), and the camera has to land on the tag's own LEFT,
     * which is negative y in the output frame — the viewer's right is the tag's left,
     * the same way it is for a person facing you.
     */
    @Test
    public void cameraOffsetToViewersRightOfTag() {
        // Camera at (200, 0, -1000) in the tag frame; tag-in-camera is -R * that.
        TagObservation observation = TagObservation.fromOpenCvPose(
                24, -200, 0, 1000, identity());

        Position camRelTag = observation.getCamRelTagPos();
        assertEquals(1000, camRelTag.getX(), MM_TOLERANCE);
        assertEquals(-200, camRelTag.getY(), MM_TOLERANCE);
        assertEquals(180, Math.abs(Math.toDegrees(camRelTag.getTheta())), DEG_TOLERANCE);
        assertEquals(0, observation.getRawPitchDeg(), DEG_TOLERANCE);
    }

    /**
     * Camera still 1000mm in front of the tag, but yawed 30 degrees about the vertical.
     *
     * rawPitchDeg has to read back as that 30 degrees, because that is exactly the number
     * the Limelight reports as camera-in-target-space pitch and exactly the number
     * AprilTagDetectionAction's 90-degree spike gate is calibrated against.
     */
    @Test
    public void cameraYawedAboutTheVertical() {
        double yawRad = Math.toRadians(30);
        // R maps tag -> camera and is the transpose of the camera's rotation in tag space.
        MatrixF tagInCameraRotation = rotationAboutY(-yawRad);

        // Camera at (0, 0, -1000) in tag coords, so tag-in-camera = -R * camPosInTag.
        double camInTagZ = -1000;
        double tagInCamX = -(tagInCameraRotation.get(0, 2) * camInTagZ);
        double tagInCamY = -(tagInCameraRotation.get(1, 2) * camInTagZ);
        double tagInCamZ = -(tagInCameraRotation.get(2, 2) * camInTagZ);

        TagObservation observation = TagObservation.fromOpenCvPose(
                20, tagInCamX, tagInCamY, tagInCamZ, tagInCameraRotation);

        Position camRelTag = observation.getCamRelTagPos();
        assertEquals(1000, camRelTag.getX(), 1e-3);
        assertEquals(0, camRelTag.getY(), 1e-3);
        assertEquals(30, observation.getRawPitchDeg(), 1e-3);
        assertEquals(-150, Math.toDegrees(camRelTag.getTheta()), 1e-3);
    }

    /**
     * The raw tag-in-camera millimetres pass through untouched. AprilTagDetectionAction
     * aims the turret off these directly (x sideways, z forward), so any rescaling or
     * reordering here would move the shot.
     */
    @Test
    public void rawCameraSpaceValuesPassThroughUnchanged() {
        TagObservation observation = TagObservation.fromOpenCvPose(
                20, 123.5, -45.25, 987.75, identity());

        assertEquals(123.5, observation.getTagRelCamXMM(), MM_TOLERANCE);
        assertEquals(-45.25, observation.getTagRelCamYMM(), MM_TOLERANCE);
        assertEquals(987.75, observation.getTagRelCamZMM(), MM_TOLERANCE);
    }

    /**
     * Round trip: pick an arbitrary camera pose in the tag frame, project it into the
     * solver's output, and confirm the conversion recovers the pose it started from.
     * This is the case that would catch a transpose error that the axis-aligned cases
     * above happen to be blind to.
     */
    @Test
    public void recoversAnArbitraryCameraPose() {
        double yawRad = Math.toRadians(-42);
        double camInTagX = 350;
        double camInTagY = -80;   // camera mounted above the tag centre (+Y is down)
        double camInTagZ = -1400; // in front of the tag face

        MatrixF camInTagRotation = rotationAboutY(yawRad);
        MatrixF tagInCamRotation = rotationAboutY(-yawRad); // transpose of the above

        // tagInCam = -R * camInTag, with R = tagInCamRotation.
        double[] camInTag = {camInTagX, camInTagY, camInTagZ};
        double[] tagInCam = new double[3];
        for (int row = 0; row < 3; row++) {
            double sum = 0;
            for (int col = 0; col < 3; col++) {
                sum += tagInCamRotation.get(row, col) * camInTag[col];
            }
            tagInCam[row] = -sum;
        }

        TagObservation observation = TagObservation.fromOpenCvPose(
                24, tagInCam[0], tagInCam[1], tagInCam[2], tagInCamRotation);

        Position camRelTag = observation.getCamRelTagPos();
        assertEquals(-camInTagZ, camRelTag.getX(), 1e-2);
        assertEquals(-camInTagX, camRelTag.getY(), 1e-2);
        assertEquals(-42, observation.getRawPitchDeg(), 1e-3);

        // Sanity: the rotation used to build the input really is the transpose of the
        // camera's own rotation in tag space, which is what the conversion assumes.
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                assertEquals(camInTagRotation.get(row, col),
                        tagInCamRotation.get(col, row), 1e-6);
            }
        }
    }
}
