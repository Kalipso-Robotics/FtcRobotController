package org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

/**
 * Known field position of every AprilTag the robot cares about, keyed by tag ID.
 * Independent of alliance, camera, or gimbal angle - it just answers "where on the
 * field is tag N mounted".
 *
 * THIS IS THE PER-TAG TRANSFORM TABLE.
 *   AprilTagDetectionAction runs one transform chain, robot -> turret -> camera -> tag ->
 *   field, and the last link is whatever this layout returns for the tag it just saw. So
 *   adding a tag here is the whole job of teaching the robot to relocalize off it: no new
 *   code path, no new action, no branching on tag ID anywhere.
 *
 *   AprilTagFieldLayout layout = new AprilTagFieldLayout()
 *           .put(24, new Position(3076.6,  1013.5, Math.toRadians(-125.54)))  // red goal
 *           .put(20, new Position(3076.6, -1013.5, Math.toRadians( 125.54)))  // blue goal
 *           .put(22, new Position(-1200,      600, Math.toRadians(   0.00))); // obelisk
 *
 * A game-specific config builds one of these (see decode.configs.AprilTagConfig
 * #buildFieldLayout). A tag NOT in the layout is detected but ignored for localization -
 * that is the intended way to keep an unmeasured tag from poisoning the pose.
 *
 * MEASURE, DO NOT GUESS: a tag entered at the wrong field pose does not fail loudly. It
 * relocalizes the robot confidently into the wrong place.
 */
public class AprilTagFieldLayout {

    // Insertion-ordered so telemetry and logs list tags the way the config declares them.
    private final Map<Integer, Position> tagFieldPoses = new LinkedHashMap<>();

    /**
     * Registers where a tag sits on the field.
     *
     * @param fieldPos the tag's origin in field coordinates, with theta being the heading
     *                 its printed face looks along (+X forward, +Y right, CCW radians)
     */
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

    /** Every tag this layout can localize off, in declaration order. */
    public Set<Integer> getKnownTagIds() {
        return Collections.unmodifiableSet(tagFieldPoses.keySet());
    }

    public int size() {
        return tagFieldPoses.size();
    }
}
