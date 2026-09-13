package org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;

/**
 * The sanity gauntlet a single tag's pose estimate has to survive before the robot is
 * allowed to believe it, plus the "seen it N frames running" confidence counter.
 *
 * ONE FILTER PER TAG, AND WHY THAT MATTERS:
 *   Every gate here is a comparison against what THIS tag looked like last frame. Share a
 *   filter between two tags and the comparisons become nonsense: tag 20's pitch measured
 *   against tag 24's pitch is a ~100 degree jump for no reason, so the robot would throw
 *   away two perfectly good fixes for the crime of seeing both tags at once.
 *   AprilTagDetectionAction therefore keeps a filter per tag ID.
 *
 * THE GATES, IN ORDER (order matters - see passedGeometryGates):
 *   1. PITCH_JUMP             the camera cannot swing 90 degrees around a tag between frames
 *   2. OFF_FIELD              a pose outside the field is arithmetic, not localization
 *   3. TELEPORT               the robot cannot move 1.2m between frames
 *   4. ODOMETRY_DISAGREEMENT  vision and the wheels disagree by more than 600mm
 *
 * The first three are geometry checks on the reading itself; the fourth cross-examines it
 * against a second sensor. AprilTagDetectionAction publishes the "unfiltered" pose for
 * anything that clears the first three, so a tuning session can see what odometry rejected.
 *
 * Deliberately free of Android and hardware imports so it can be unit tested.
 */
public final class TagPoseFilter {

    /**
     * Outcome of one frame's evaluation for one tag.
     *
     * ACCEPTED means "use this". WARMING_UP means the reading passed every gate but the
     * tag has not been seen cleanly enough times in a row yet - the pose is published,
     * the goal aim is not, because a shot taken off one lucky frame is worse than no shot.
     */
    public enum Verdict {
        ACCEPTED(false, true),
        WARMING_UP(false, true),
        PITCH_JUMP(true, false),
        OFF_FIELD(true, false),
        TELEPORT(true, false),
        ODOMETRY_DISAGREEMENT(true, true);

        private final boolean rejection;
        private final boolean passedGeometry;

        Verdict(boolean rejection, boolean passedGeometry) {
            this.rejection = rejection;
            this.passedGeometry = passedGeometry;
        }

        /** True when this reading must not be used to update the robot's position. */
        public boolean isRejection() { return rejection; }

        /** True only when the reading is good enough to aim off, not just localize with. */
        public boolean isUsable() { return this == ACCEPTED; }

        /**
         * True when the reading cleared the three geometry gates, whether or not odometry
         * later disagreed with it. This is the cut for the "unfiltered" diagnostic pose.
         */
        public boolean passedGeometryGates() { return passedGeometry; }
    }

    // Thresholds are static and public so a tuning OpMode can drive them live. They carry
    // the values the Limelight pipeline was tuned with - change them with data, not vibes.

    /** Clean frames in a row before a tag is trusted for aiming. */
    public static int STABILITY_THRESHOLD = 2;

    /**
     * Max frame-to-frame change in the camera's angle around the tag. Also acts as a
     * first-sighting gate: the filter starts at 0 degrees, and |pitch| > 90 means the
     * camera is edge-on or behind the tag plane, which cannot happen while the tag is
     * readable, so a first reading that extreme is a solver failure.
     */
    public static double MAX_PITCH_JUMP_DEG = 90;

    /** Field bounds. A pose outside these came from bad math, not a bad camera. */
    public static double FIELD_HALF_LENGTH_MM = 3600;
    public static double FIELD_HALF_WIDTH_MM = 3000;

    /** Max distance the robot may appear to have moved since this tag's last good fix. */
    public static double MAX_JUMP_FROM_PREV_MM = 1200;

    /** Max distance vision may disagree with the odometry pose before it is discarded. */
    public static double MAX_ODOMETRY_DISAGREEMENT_MM = 600;

    private final int tagId;

    /**
     * Last pitch SEEN for this tag, accepted or not. Anchoring on the last seen reading
     * rather than the last accepted one is what lets the filter re-latch after a burst of
     * garbage instead of measuring every new frame against an increasingly stale good one.
     */
    private double prevPitchDeg = 0;

    /** Last ACCEPTED pose for this tag. Null disables the teleport gate for one frame. */
    private Position prevGlobalPos;

    private int consecutiveGoodReadings = 0;
    private Verdict lastVerdict = null;

    public TagPoseFilter(int tagId) {
        this.tagId = tagId;
    }

    public int getTagId() { return tagId; }

    /**
     * Runs one candidate pose for this tag through every gate and updates the filter's
     * memory. Call exactly once per tag per frame.
     *
     * @param rawPitchDeg  the observation's raw pitch, from TagObservation
     * @param globalPos    the field pose this tag's transform produced this frame
     * @param odometryPos  the wheel/IMU pose to cross-check against; null skips that gate
     */
    public Verdict evaluate(double rawPitchDeg, Position globalPos, Position odometryPos) {
        if (globalPos == null) {
            return reject(rawPitchDeg, Verdict.OFF_FIELD);
        }

        double pitchDeltaDeg = MathFunctions.angleWrapDeg(rawPitchDeg - prevPitchDeg);
        if (Math.abs(pitchDeltaDeg) > MAX_PITCH_JUMP_DEG) {
            return reject(rawPitchDeg, Verdict.PITCH_JUMP);
        }

        if (Math.abs(globalPos.getX()) > FIELD_HALF_LENGTH_MM
                || Math.abs(globalPos.getY()) > FIELD_HALF_WIDTH_MM) {
            return reject(rawPitchDeg, Verdict.OFF_FIELD);
        }

        if (prevGlobalPos != null && !prevGlobalPos.isEmpty()
                && globalPos.distanceTo(prevGlobalPos) > MAX_JUMP_FROM_PREV_MM) {
            return reject(rawPitchDeg, Verdict.TELEPORT);
        }

        if (odometryPos != null
                && globalPos.distanceTo(odometryPos) > MAX_ODOMETRY_DISAGREEMENT_MM) {
            return reject(rawPitchDeg, Verdict.ODOMETRY_DISAGREEMENT);
        }

        prevPitchDeg = rawPitchDeg;
        prevGlobalPos = globalPos;
        // Clamped so a tag parked in view for a whole match cannot overflow the counter.
        consecutiveGoodReadings = Math.min(consecutiveGoodReadings + 1, STABILITY_THRESHOLD + 1);

        lastVerdict = (consecutiveGoodReadings > STABILITY_THRESHOLD)
                ? Verdict.ACCEPTED
                : Verdict.WARMING_UP;
        return lastVerdict;
    }

    private Verdict reject(double rawPitchDeg, Verdict verdict) {
        prevPitchDeg = rawPitchDeg;
        // Forgetting the previous fix disables the teleport gate on the next frame. That is
        // deliberate: after a genuine relocation the old pose is the wrong thing to compare
        // against, and keeping it would lock the tag out permanently.
        prevGlobalPos = null;
        consecutiveGoodReadings = 0;
        lastVerdict = verdict;
        return verdict;
    }

    /** Forget everything about this tag, e.g. after the robot is manually repositioned. */
    public void reset() {
        prevPitchDeg = 0;
        prevGlobalPos = null;
        consecutiveGoodReadings = 0;
        lastVerdict = null;
    }

    /** Verdict from the last evaluate() call, or null if this tag has never been seen. */
    public Verdict getLastVerdict() { return lastVerdict; }

    public int getConsecutiveGoodReadings() { return consecutiveGoodReadings; }

    /** Last accepted field pose for this tag, or null if it has none. */
    public Position getPrevGlobalPos() { return prevGlobalPos; }

    public double getPrevPitchDeg() { return prevPitchDeg; }
}
