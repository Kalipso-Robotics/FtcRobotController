package org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;

/**
 * A camera that's rigidly bolted to the turret with no pan/tilt of its own - this
 * season's rig. Always returns the same fixed offset, so it's the default CameraGimbal
 * for AprilTagDetectionAction until a real CameraMount is wired in.
 */
public class FixedCameraMount implements CameraGimbal {

    private final Position turretRelCamPos;

    public FixedCameraMount(Position turretRelCamPos) {
        this.turretRelCamPos = turretRelCamPos;
    }

    @Override
    public Position getMountRelCamPos() {
        return turretRelCamPos;
    }
}
