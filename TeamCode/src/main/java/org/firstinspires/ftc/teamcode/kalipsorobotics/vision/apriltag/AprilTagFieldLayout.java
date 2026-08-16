package org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;

import java.util.HashMap;
import java.util.Map;

/**
 * Known field position of every AprilTag the robot cares about, keyed by tag ID.
 * Independent of alliance, camera, or gimbal angle - it just answers "where on the
 * field is tag N mounted". A game-specific config builds one of these (see
 * decode.configs.AprilTagConfig#buildFieldLayout), and AprilTagDetectionAction resolves
 * whichever tag it observes against it. This is what lets a single action localize off
 * any tag the 2-servo gimbal happens to be pointed at, instead of one hardcoded target.
 */
public class AprilTagFieldLayout {

    private final Map<Integer, Position> tagFieldPoses = new HashMap<>();

    public AprilTagFieldLayout put(int tagId, Position fieldPos) {
        tagFieldPoses.put(tagId, fieldPos);
        return this;
    }

    /** Returns the tag's field position, or null if this tag isn't part of the known layout. */
    public Position getFieldPose(int tagId) {
        return tagFieldPoses.get(tagId);
    }

    public boolean isKnown(int tagId) {
        return tagFieldPoses.containsKey(tagId);
    }
}
