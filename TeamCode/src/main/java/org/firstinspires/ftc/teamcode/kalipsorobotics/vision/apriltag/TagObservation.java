package org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;

/**
 * A single AprilTag detection, already normalized into the field/robot axis
 * convention (mm, radians) that AprilTagDetectionAction expects. Every
 * AprilTagCamera implementation is responsible for producing these, so the
 * action itself never has to know which camera/SDK produced the detection.
 */
public final class TagObservation {

    private final int tagId;
    private final Position camRelTagPos;
    private final double rawPitchDeg;
    private final double tagRelCamXMM;
    private final double tagRelCamYMM;
    private final double tagRelCamZMM;

    public TagObservation(int tagId, Position camRelTagPos, double rawPitchDeg,
                           double tagRelCamXMM, double tagRelCamYMM, double tagRelCamZMM) {
        this.tagId = tagId;
        this.camRelTagPos = camRelTagPos;
        this.rawPitchDeg = rawPitchDeg;
        this.tagRelCamXMM = tagRelCamXMM;
        this.tagRelCamYMM = tagRelCamYMM;
        this.tagRelCamZMM = tagRelCamZMM;
    }

    /**
     * Builds a TagObservation from a pose expressed in the OpenCV / AprilTag convention,
     * which is what every non-Limelight solver hands back (the FTC SDK's
     * {@code AprilTagDetection.rawPose}, a bare {@code solvePnP}, etc.).
     *
     * <p>INPUT CONVENTION (both frames are OpenCV-standard):
     * <ul>
     *   <li><b>Camera frame</b>: +X right, +Y down, +Z forward along the optical axis.</li>
     *   <li><b>Tag frame</b>: +X right and +Y down as seen by someone looking at the tag,
     *       +Z through the tag away from that viewer.</li>
     * </ul>
     * This is exactly the Limelight's "camera space" / "target space" pair, which is why
     * LimelightAprilTagCamera and any solver-backed camera can share one conversion.
     *
     * <p>OUTPUT CONVENTION (this codebase's planar frames, see AprilTagDetectionAction):
     * every frame is (x = forward, y = right, theta = CCW). For the camera, "forward" is
     * the optical axis; for the tag, "forward" is out of its printed face toward the
     * viewer. So the camera's pose in the tag's frame comes out as:
     * <pre>
     *   x     = -camInTagZ    (how far in FRONT of the tag face the camera sits)
     *   y     = -camInTagX    (offset along the tag's own right)
     *   theta = heading of the camera's optical axis in that plane
     *           (180 deg when the camera stares straight at the tag)
     * </pre>
     *
     * <p>Only the planar component is kept: tilt and roll are dropped, exactly as the
     * Limelight path does, because the localization chain in AprilTagDetectionAction is
     * a 2D (x, y, theta) transform stack.
     *
     * @param tagId               the detected tag's ID
     * @param tagInCamXMM         tag center in the camera frame, +X right, mm
     * @param tagInCamYMM         tag center in the camera frame, +Y down, mm
     * @param tagInCamZMM         tag center in the camera frame, +Z forward, mm
     * @param tagRotationInCamera 3x3 rotation taking TAG-frame vectors to CAMERA-frame
     *                            vectors (i.e. {@code AprilTagDetection.rawPose.R})
     */
    public static TagObservation fromOpenCvPose(int tagId,
                                                double tagInCamXMM,
                                                double tagInCamYMM,
                                                double tagInCamZMM,
                                                MatrixF tagRotationInCamera) {
        // R maps tag -> camera, so its transpose maps camera -> tag.
        // Camera origin in the tag frame: c = -R^T * t
        double camInTagX = 0, camInTagY = 0, camInTagZ = 0;
        for (int row = 0; row < 3; row++) {
            double tComponent = (row == 0) ? tagInCamXMM : (row == 1) ? tagInCamYMM : tagInCamZMM;
            camInTagX -= tagRotationInCamera.get(row, 0) * tComponent;
            camInTagY -= tagRotationInCamera.get(row, 1) * tComponent;
            camInTagZ -= tagRotationInCamera.get(row, 2) * tComponent;
        }

        // Camera optical axis (0,0,1 in camera coords) expressed in tag coords is
        // R^T * (0,0,1), which is row 2 of R.
        double opticalAxisInTagX = tagRotationInCamera.get(2, 0);
        double opticalAxisInTagZ = tagRotationInCamera.get(2, 2);

        // Project that axis onto the tag's planar frame (x = -Z_tag, y = -X_tag) and read
        // off its heading. Degenerate only if the camera looks straight along the tag's
        // vertical axis, which cannot happen while the tag is in frame.
        double camRelTagTheta = MathFunctions.angleWrapRad(
                Math.atan2(-opticalAxisInTagX, -opticalAxisInTagZ));

        Position camRelTagPos = new Position(-camInTagZ, -camInTagX, camRelTagTheta);

        // The Limelight path feeds spike detection with the raw pitch of the camera in tag
        // space, which is this same planar angle minus the 180 deg "staring at the tag"
        // offset. Deriving it here keeps both cameras on one scale, so the thresholds in
        // AprilTagDetectionAction mean the same thing either way.
        double rawPitchDeg = MathFunctions.angleWrapDeg(Math.toDegrees(camRelTagTheta) - 180);

        return new TagObservation(tagId, camRelTagPos, rawPitchDeg,
                tagInCamXMM, tagInCamYMM, tagInCamZMM);
    }

    public int getTagId() {
        return tagId;
    }

    /**
     * Camera's pose relative to the tag, in the (x=-Z, y=-X, theta=180+pitch) convention
     * used by the robot->turret->cam->tag->field transform chain.
     */
    public Position getCamRelTagPos() {
        return camRelTagPos;
    }

    /** Raw pitch of the camera-relative-to-tag orientation, used for spike detection. */
    public double getRawPitchDeg() {
        return rawPitchDeg;
    }

    public double getTagRelCamXMM() {
        return tagRelCamXMM;
    }

    public double getTagRelCamYMM() {
        return tagRelCamYMM;
    }

    public double getTagRelCamZMM() {
        return tagRelCamZMM;
    }

    /**
     * Straight-line distance from the camera to the tag across the ground plane, mm.
     *
     * The vertical component is deliberately left out: it is the same for every tag on a
     * given mount and only inflates the number. This is what AprilTagDetectionAction
     * ranges the goal with, and what it compares tags by when several are in view at once
     * - a nearer tag gives a better pose, because the same angular error in the solver
     * translates into less position error the closer you are.
     */
    public double getGroundRangeMM() {
        return Math.hypot(tagRelCamXMM, tagRelCamZMM);
    }

    @Override
    public String toString() {
        return "TagObservation{" +
                "tagId=" + tagId +
                ", camRelTagPos=" + camRelTagPos +
                ", rawPitchDeg=" + rawPitchDeg +
                ", tagRelCam=(" + tagRelCamXMM + ", " + tagRelCamYMM + ", " + tagRelCamZMM + ")" +
                '}';
    }
}
