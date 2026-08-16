package org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag;

import java.util.List;

/**
 * Hardware-agnostic source of AprilTag detections. Implementations own everything
 * specific to the physical camera/SDK (pipeline config, pose-frame conversion, etc.)
 * and hand back normalized TagObservations. This is the seam that lets
 * AprilTagDetectionAction move from the Limelight 3A to the Arducam OV9782 (or any
 * future camera) without changing its detection/filtering logic.
 */
public interface AprilTagCamera {

    void start();

    void stop();

    /** Returns all tags visible in the latest frame, or an empty list if none/invalid. */
    List<TagObservation> getLatestObservations();
}
