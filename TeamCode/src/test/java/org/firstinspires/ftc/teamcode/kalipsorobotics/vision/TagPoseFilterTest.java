package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AprilTagFieldLayout;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.TagPoseFilter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.TagPoseFilter.Verdict;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/**
 * Covers the gate logic AprilTagDetectionAction relies on to decide which of several
 * simultaneously visible tags it is allowed to relocalize from.
 *
 * The case that motivated splitting the filter out per tag is
 * {@link #twoTagsDoNotPoisonEachOther()} - with one shared filter, seeing two tags at once
 * makes each one look like a violent spike to the other and the robot throws away both.
 */
public class TagPoseFilterTest {

    private static final int RED_GOAL = 24;
    private static final int BLUE_GOAL = 20;

    /** Somewhere sane on the field, matching the odometry pose used below. */
    private static Position pose(double x, double y) {
        return new Position(x, y, 0);
    }

    private static final Position ODOMETRY = pose(1000, 500);

    private TagPoseFilter filter;

    @Before
    public void setUp() {
        filter = new TagPoseFilter(RED_GOAL);
    }

    /** Feeds the same clean reading until the filter promotes it to ACCEPTED. */
    private Verdict settle(TagPoseFilter target, double pitchDeg, Position pos) {
        Verdict verdict = null;
        for (int i = 0; i <= TagPoseFilter.STABILITY_THRESHOLD; i++) {
            verdict = target.evaluate(pitchDeg, pos, ODOMETRY);
        }
        return verdict;
    }

    // -------------------------------------------------------------------------
    // Warm-up
    // -------------------------------------------------------------------------

    /**
     * A single good frame is not enough to aim off. The pose is still published (the
     * action treats WARMING_UP as non-rejected) but the turret waits, because a shot taken
     * off one lucky frame is worse than no shot.
     */
    @Test
    public void firstCleanReadingIsWarmingUpNotAccepted() {
        assertEquals(Verdict.WARMING_UP, filter.evaluate(0, ODOMETRY, ODOMETRY));
        assertFalse(filter.evaluate(0, ODOMETRY, ODOMETRY).isUsable());
        assertEquals(Verdict.ACCEPTED, filter.evaluate(0, ODOMETRY, ODOMETRY));
    }

    @Test
    public void warmingUpIsNotARejection() {
        assertFalse(Verdict.WARMING_UP.isRejection());
        assertTrue(Verdict.WARMING_UP.passedGeometryGates());
    }

    @Test
    public void neverSeenTagHasNoVerdict() {
        assertNull(filter.getLastVerdict());
        assertEquals(0, filter.getConsecutiveGoodReadings());
    }

    // -------------------------------------------------------------------------
    // The four gates
    // -------------------------------------------------------------------------

    /**
     * A tag first sighted at an impossible angle is rejected. |pitch| > 90 means the camera
     * is edge-on or behind the tag plane, where the tag would not be readable at all, so
     * such a reading is a solver failure rather than a real sighting.
     */
    @Test
    public void impossibleFirstPitchIsRejected() {
        assertEquals(Verdict.PITCH_JUMP, filter.evaluate(150, ODOMETRY, ODOMETRY));
    }

    @Test
    public void pitchJumpBetweenFramesIsRejected() {
        settle(filter, 10, ODOMETRY);
        assertEquals(Verdict.PITCH_JUMP, filter.evaluate(120, ODOMETRY, ODOMETRY));
    }

    @Test
    public void poseOutsideTheFieldIsRejected() {
        assertEquals(Verdict.OFF_FIELD,
                filter.evaluate(0, pose(TagPoseFilter.FIELD_HALF_LENGTH_MM + 1, 0), ODOMETRY));
        assertEquals(Verdict.OFF_FIELD,
                filter.evaluate(0, pose(0, TagPoseFilter.FIELD_HALF_WIDTH_MM + 1), ODOMETRY));
    }

    @Test
    public void nullPoseIsRejectedRatherThanCrashing() {
        assertEquals(Verdict.OFF_FIELD, filter.evaluate(0, null, ODOMETRY));
    }

    /** The robot cannot cross 1.2m of field between two camera frames. */
    @Test
    public void teleportFromThePreviousFixIsRejected() {
        settle(filter, 0, ODOMETRY);
        Position farAway = pose(ODOMETRY.getX() + TagPoseFilter.MAX_JUMP_FROM_PREV_MM + 1,
                ODOMETRY.getY());
        assertEquals(Verdict.TELEPORT, filter.evaluate(0, farAway, farAway));
    }

    /**
     * Vision and the wheels disagreeing by more than 600mm means one of them is lying, and
     * the safe move is to keep the odometry pose rather than teleport the robot.
     */
    @Test
    public void disagreementWithOdometryIsRejected() {
        Position visionSaysHere = pose(2500, 500);
        assertEquals(Verdict.ODOMETRY_DISAGREEMENT,
                filter.evaluate(0, visionSaysHere, ODOMETRY));
    }

    /**
     * The odometry gate is the last one, so a pose it rejects has still cleared the
     * geometry checks. The action publishes those on the unfiltered channel, which is the
     * only way to see on the dashboard what odometry is throwing away.
     */
    @Test
    public void odometryRejectionStillCountsAsPassingGeometry() {
        assertTrue(Verdict.ODOMETRY_DISAGREEMENT.passedGeometryGates());
        assertTrue(Verdict.ODOMETRY_DISAGREEMENT.isRejection());
        assertFalse(Verdict.PITCH_JUMP.passedGeometryGates());
        assertFalse(Verdict.OFF_FIELD.passedGeometryGates());
        assertFalse(Verdict.TELEPORT.passedGeometryGates());
    }

    @Test
    public void nullOdometrySkipsThatGate() {
        // Nothing to cross-check against, so a pose far from the (absent) odometry passes.
        assertEquals(Verdict.WARMING_UP, filter.evaluate(0, pose(2500, 500), null));
    }

    // -------------------------------------------------------------------------
    // Recovery
    // -------------------------------------------------------------------------

    /** A rejection wipes the confidence counter, so aiming has to earn trust again. */
    @Test
    public void rejectionResetsStability() {
        settle(filter, 0, ODOMETRY);
        assertTrue(filter.getLastVerdict().isUsable());

        filter.evaluate(120, ODOMETRY, ODOMETRY); // PITCH_JUMP
        assertEquals(0, filter.getConsecutiveGoodReadings());
        assertEquals(Verdict.WARMING_UP, filter.evaluate(120, ODOMETRY, ODOMETRY));
    }

    /**
     * After a rejection the teleport gate must not fire on the very next frame. The old
     * fix is no longer a valid thing to compare against, and keeping it would lock a tag
     * out forever once the robot was genuinely repositioned.
     */
    @Test
    public void rejectionClearsThePreviousFixSoTheTagCanRelatch() {
        settle(filter, 0, ODOMETRY);
        assertSame(ODOMETRY, filter.getPrevGlobalPos());

        filter.evaluate(0, pose(9999, 0), ODOMETRY); // OFF_FIELD
        assertNull(filter.getPrevGlobalPos());

        // A pose miles from the old fix is now accepted, since there is no old fix.
        Position elsewhere = pose(2000, -400);
        assertEquals(Verdict.WARMING_UP, filter.evaluate(0, elsewhere, elsewhere));
    }

    /**
     * The pitch gate compares against the last pitch SEEN, not the last one accepted. If
     * it anchored on the last accepted reading, a camera that swung round a tag while
     * producing junk would be measured against an ever-staler angle and could never
     * recover once the readings turned good again.
     */
    @Test
    public void pitchGateFollowsRejectedReadings() {
        settle(filter, 0, ODOMETRY);
        filter.evaluate(120, ODOMETRY, ODOMETRY); // rejected, but 120 is now the anchor
        assertEquals(120, filter.getPrevPitchDeg(), 1e-9);
        // 130 is a 10 degree step from the rejected reading, so it passes.
        assertEquals(Verdict.WARMING_UP, filter.evaluate(130, ODOMETRY, ODOMETRY));
    }

    @Test
    public void resetForgetsEverything() {
        settle(filter, 45, ODOMETRY);
        filter.reset();
        assertNull(filter.getLastVerdict());
        assertNull(filter.getPrevGlobalPos());
        assertEquals(0, filter.getConsecutiveGoodReadings());
        assertEquals(0, filter.getPrevPitchDeg(), 1e-9);
    }

    // -------------------------------------------------------------------------
    // Why the filter is per-tag
    // -------------------------------------------------------------------------

    /**
     * The whole reason AprilTagDetectionAction keeps a Map<tagId, TagPoseFilter>.
     *
     * Two goal tags face opposite directions, so the camera's angle around one differs
     * from its angle around the other by far more than the 90 degree gate. Run both
     * through ONE filter and each frame looks like a violent spike caused by the other
     * tag, and the robot discards two good fixes. Run them through their own filters and
     * both settle independently.
     */
    @Test
    public void twoTagsDoNotPoisonEachOther() {
        TagPoseFilter red = new TagPoseFilter(RED_GOAL);
        TagPoseFilter blue = new TagPoseFilter(BLUE_GOAL);

        // Same frames, alternating tags. Each pitch is plausible on its own (both within
        // 90 degrees of head-on) but they are 120 degrees apart from each other.
        for (int frame = 0; frame <= TagPoseFilter.STABILITY_THRESHOLD; frame++) {
            red.evaluate(60, ODOMETRY, ODOMETRY);
            blue.evaluate(-60, ODOMETRY, ODOMETRY);
        }

        assertEquals(Verdict.ACCEPTED, red.getLastVerdict());
        assertEquals(Verdict.ACCEPTED, blue.getLastVerdict());

        // The same sequence through a single shared filter is rejected outright, which is
        // the failure this design avoids.
        TagPoseFilter shared = new TagPoseFilter(RED_GOAL);
        shared.evaluate(60, ODOMETRY, ODOMETRY);
        assertEquals(Verdict.PITCH_JUMP, shared.evaluate(-60, ODOMETRY, ODOMETRY));
    }

    // -------------------------------------------------------------------------
    // The layout is the per-tag transform table
    // -------------------------------------------------------------------------

    @Test
    public void layoutResolvesEachTagToItsOwnFieldPose() {
        Position redPose = new Position(3076.6, 1013.5, Math.toRadians(-125.54));
        Position bluePose = new Position(3076.6, -1013.5, Math.toRadians(125.54));

        AprilTagFieldLayout layout = new AprilTagFieldLayout()
                .put(RED_GOAL, redPose)
                .put(BLUE_GOAL, bluePose);

        assertSame(redPose, layout.getFieldPose(RED_GOAL));
        assertSame(bluePose, layout.getFieldPose(BLUE_GOAL));
        assertTrue(layout.isKnown(BLUE_GOAL));
        assertEquals(2, layout.size());

        // An unmeasured tag resolves to nothing, which is how the action knows to ignore
        // it rather than relocalize off a guess.
        assertNull(layout.getFieldPose(22));
        assertFalse(layout.isKnown(22));
        assertFalse(layout.getKnownTagIds().contains(22));
    }
}
