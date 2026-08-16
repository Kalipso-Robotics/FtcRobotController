package org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;

/**
 * Where the AprilTag camera currently sits relative to whatever it's bolted to (the
 * turret), given however the mount is currently posed. This is the seam that lets
 * AprilTagDetectionAction go from "camera is rigidly fixed to the turret" (this season,
 * a constant offset) to "camera sits on a 2-servo pan/tilt gimbal" (next season, an
 * offset that moves as the servos move) without changing the localization math.
 */
public interface CameraGimbal {

    /**
     * Camera's position in the turret's frame, accounting for the mount's current pose.
     * Only the planar (x, y, theta) component is meaningful here — see CameraMount for
     * why tilt doesn't factor into this.
     */
    Position getMountRelCamPos();
}
