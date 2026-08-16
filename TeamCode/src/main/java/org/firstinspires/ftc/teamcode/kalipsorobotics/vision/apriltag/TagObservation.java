package org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag;

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
