package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.AprilTagConfig.*;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AprilTagCamera;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AprilTagFieldLayout;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.CameraGimbal;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.FixedCameraMount;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.LimelightAprilTagCamera;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.TagObservation;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.TagPoseFilter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.LimelightPos;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Turns AprilTag sightings into a field position, and the goal's bearing/range, on
 * SharedData. All the camera-specific work happens behind AprilTagCamera, so this class
 * only ever sees normalized TagObservations and never learns which hardware produced them.
 *
 * ANY KNOWN TAG RELOCALIZES THE ROBOT:
 *   Every frame, every visible tag that appears in the AprilTagFieldLayout is processed.
 *   The transform chain is always the same shape - robot -> turret -> camera -> tag ->
 *   field - and the last link is that tag's own field pose, looked up by ID. So the
 *   per-tag transform IS the layout entry: teaching the robot a new tag is one
 *   layout.put(id, fieldPose) line in AprilTagConfig, with no change here.
 *
 *   Concretely, a red robot that can see the blue alliance's goal tag now relocalizes off
 *   it, because both goal tags are in the layout. Before, everything but one hardcoded ID
 *   was discarded.
 *
 * WHEN SEVERAL TAGS ARE VISIBLE AT ONCE:
 *   Each tag is filtered on its own history (see TagPoseFilter - sharing one filter across
 *   tags would make every tag look like a spike to every other tag), and the surviving fix
 *   from the NEAREST tag wins. Nearest, because the solver's angular error turns into less
 *   position error the closer the tag is. The rejected candidates are logged, not silently
 *   dropped, so a bad layout entry shows up as one tag disagreeing with the rest.
 *
 * THE GOAL TAG IS STILL SPECIAL, BUT ONLY FOR AIMING:
 *   Localization uses any known tag. Turret aiming (SharedData's LimelightPos) only ever
 *   comes from the goal tag passed to the constructor, since the goal offset is measured
 *   relative to that specific tag. Losing sight of the goal tag clears the aim data even
 *   while another tag keeps the robot localized - stale aim is worse than no aim.
 *
 * PICKING A CAMERA:
 *   // Limelight 3A - the default, needs no extra wiring
 *   new AprilTagDetectionAction(opModeUtilities, turret, tagId, alliance);
 *
 *   // Webcam on the shared VisionPortal, so the same frames also feed the blob processors
 *   KAprilTagProcessor tags = new KAprilTagProcessor.Builder().build();
 *   VisionManager camera = new VisionManager.Builder(hardwareMap)
 *           .addProcessor(tags)
 *           .addProcessor(new ArtifactColorBlobDetectionProcessor())
 *           .streamImmediately()
 *           .build();
 *   new AprilTagDetectionAction(opModeUtilities, turret, tagId, alliance, tags);
 */
public class AprilTagDetectionAction extends Action {

    /** Returned by getLastFixTagId() when no tag produced a usable fix on the last update. */
    public static final int NO_TAG = -1;

    AllianceColor allianceColor;

    private final OpModeUtilities opModeUtilities;

    private final AprilTagCamera camera;
    private final CameraGimbal cameraGimbal;
    private final Turret turret;

    /** The tag the turret aims off. Localization is NOT restricted to this one. */
    private final int goalAprilTagId;

    private final AprilTagFieldLayout fieldLayout;

    /**
     * One spike/stability gate per tag ID, created on first sighting. Per-tag because
     * every gate in TagPoseFilter compares against that same tag's previous frame.
     */
    private final Map<Integer, TagPoseFilter> filters = new HashMap<>();

    private boolean hasStarted = false;

    /** Which tag produced the published field position last update, or NO_TAG. */
    private int lastFixTagId = NO_TAG;
    private Position globalPos;

    /** Consecutive updates the GOAL tag has been missing or rejected. */
    private int consecutiveBadAimReadings;

    public static int CONSECUTIVE_BAD_READING_TOLERANCE = 0;

    public AprilTagDetectionAction(OpModeUtilities opModeUtilities, Turret turret, int goalAprilTagId, AllianceColor allianceColor) {
        this(opModeUtilities, turret, goalAprilTagId, allianceColor, new LimelightAprilTagCamera(opModeUtilities, allianceColor));
    }

    /**
     * Allows any AprilTagCamera implementation (e.g. an Arducam-backed one) to be
     * substituted in without touching the detection/filtering logic below. Assumes the
     * camera is rigidly bolted to the turret (this season's rig) - use the overload below
     * once a real pan/tilt CameraMount exists.
     */
    public AprilTagDetectionAction(OpModeUtilities opModeUtilities, Turret turret, int goalAprilTagId, AllianceColor allianceColor, AprilTagCamera camera) {
        this(opModeUtilities, turret, goalAprilTagId, allianceColor, camera, new FixedCameraMount(TURRET_REL_CAM_POS));
    }

    /**
     * Allows any CameraGimbal (e.g. a 2-servo pan/tilt CameraMount) to be substituted in
     * without touching the detection/filtering logic below.
     */
    public AprilTagDetectionAction(OpModeUtilities opModeUtilities, Turret turret, int goalAprilTagId, AllianceColor allianceColor, AprilTagCamera camera, CameraGimbal cameraGimbal) {
        this(opModeUtilities, turret, goalAprilTagId, allianceColor, camera, cameraGimbal, buildFieldLayout());
    }

    /**
     * Takes an explicit tag layout, which is how you add tags the season config does not
     * ship - an obelisk, a practice-field tag, a tag you taped to a wall for testing.
     * Every tag in the layout can relocalize the robot; tags outside it are ignored.
     */
    public AprilTagDetectionAction(OpModeUtilities opModeUtilities, Turret turret, int goalAprilTagId, AllianceColor allianceColor, AprilTagCamera camera, CameraGimbal cameraGimbal, AprilTagFieldLayout fieldLayout) {
        this.allianceColor = allianceColor;
        this.opModeUtilities = opModeUtilities;

        this.fieldLayout = fieldLayout;

        this.camera = camera;
        this.camera.start();
        this.cameraGimbal = cameraGimbal;

        this.turret = turret;
        this.goalAprilTagId = goalAprilTagId;

        KLog.d("AprilTag", () -> "Localizing off tags " + fieldLayout.getKnownTagIds()
                + ", aiming off tag " + goalAprilTagId);
    }

    public AprilTagCamera getCamera() {
        return camera;
    }

    @Override
    protected void update() {
        if (!hasStarted) {
            hasStarted = true;
        }

        List<TagObservation> observations = camera.getLatestObservations();
        Position odometryPos = SharedData.getOdometryWheelIMUPosition();

        // Best fix by two different bars: the strictest one drives the robot's position,
        // the looser one only feeds the unfiltered diagnostic channel so a tuning session
        // can see what the odometry cross-check threw away.
        TagFix bestAcceptedFix = null;
        TagFix bestGeometryFix = null;
        TagFix goalFix = null;

        for (TagObservation observation : observations) {
            final int tagId = observation.getTagId();

            // THE PER-TAG TRANSFORM. Same chain for every tag; the tag's own field pose is
            // the final link, so the layout entry is what makes one tag differ from another.
            Position tagFieldPos = fieldLayout.getFieldPose(tagId);
            if (tagFieldPos == null) {
                KLog.d("AprilTag", () -> "Tag " + tagId + " is not in the field layout, ignoring");
                continue;
            }

            Position candidatePos = calculateGlobalPosition(observation, tagFieldPos);
            TagPoseFilter.Verdict verdict = filterFor(tagId)
                    .evaluate(observation.getRawPitchDeg(), candidatePos, odometryPos);

            TagFix fix = new TagFix(observation, candidatePos, verdict);
            logFix(fix);

            if (tagId == goalAprilTagId) {
                goalFix = fix;
            }
            if (verdict.passedGeometryGates() && fix.isNearerThan(bestGeometryFix)) {
                bestGeometryFix = fix;
            }
            if (!verdict.isRejection() && fix.isNearerThan(bestAcceptedFix)) {
                bestAcceptedFix = fix;
            }
        }

        publishGlobalPosition(bestGeometryFix, bestAcceptedFix);
        publishGoalAim(goalFix);
    }

    /**
     * Runs the robot -> turret -> camera -> tag -> field chain for one observation.
     *
     * Coordinate system:
     * - Field: +X is forward from init position, +Y is right, angles are CCW from +X
     * - Camera: +Z is forward (optical axis), +X is right, +Y is down
     * - AprilTag's local frame: +X is the direction the tag faces, origin at tag center
     *
     * Every link but the last is tag-independent: where the turret is pointed, where the
     * camera sits on its mount, where the tag sits relative to the camera. The last link,
     * tag -> field, is the per-tag part, and it comes from the AprilTagFieldLayout.
     *
     * @param observation the tag sighting to transform
     * @param tagFieldPos where that specific tag lives on the field (from the layout)
     * @return the robot's field position implied by this one tag
     */
    public Position calculateGlobalPosition(TagObservation observation, Position tagFieldPos) {
        Position robotRelRobotPos = new Position(0, 0, 0);
        double turretAngle = turret.getCurrentAngleRad();

        Position robotRelTurretPos = robotRelRobotPos.toNewFrame(new Position(ROBOT_REL_TURRET_POINT.getX(), ROBOT_REL_TURRET_POINT.getY(), -turretAngle));
        Position robotRelCamPos = robotRelTurretPos.toNewFrame(cameraGimbal.getMountRelCamPos());
        Position robotRelAprilTagPos = robotRelCamPos.toNewFrame(observation.getCamRelTagPos());
        return robotRelAprilTagPos.toNewFrame(tagFieldPos);
    }

    // -------------------------------------------------------------------------
    // Publishing
    // -------------------------------------------------------------------------

    /**
     * @param bestGeometryFix nearest fix that cleared the geometry gates, odometry
     *                        cross-check aside. Diagnostic channel only.
     * @param bestAcceptedFix nearest fix that cleared every gate. This is what moves the
     *                        robot's believed position.
     */
    private void publishGlobalPosition(TagFix bestGeometryFix, TagFix bestAcceptedFix) {
        if (bestGeometryFix != null) {
            SharedData.setUnfilteredLimelightGlobalPos(bestGeometryFix.globalPos);
        }

        if (bestAcceptedFix == null) {
            lastFixTagId = NO_TAG;
            return;
        }

        lastFixTagId = bestAcceptedFix.observation.getTagId();
        globalPos = bestAcceptedFix.globalPos;
        SharedData.setLimelightGlobalPosition(globalPos);

        final TagFix published = bestAcceptedFix;
        KLog.d("AprilTag_GLOBAL", () -> String.format(Locale.US,
                "tag %d @ %.0fmm -> RobotPos(x=%.1fmm, y=%.1fmm, th=%.2f deg)",
                published.observation.getTagId(), published.observation.getGroundRangeMM(),
                published.globalPos.getX(), published.globalPos.getY(),
                Math.toDegrees(published.globalPos.getTheta())));
    }

    /**
     * Turret aiming, which only ever comes from the goal tag - the goal offset below is
     * measured relative to that one tag, so pointing the turret using any other tag would
     * aim at a goal that isn't there.
     *
     * @param goalFix this frame's goal-tag fix, or null if the goal tag was not in view
     */
    private void publishGoalAim(TagFix goalFix) {
        if (goalFix == null) {
            consecutiveBadAimReadings++;
            if (consecutiveBadAimReadings > CONSECUTIVE_BAD_READING_TOLERANCE) {
                KLog.d("AprilTag", "Goal tag not detected. Clearing goal aim data.");
                SharedData.getLimelightRawPosition().reset();
            }
            return;
        }

        if (goalFix.verdict.isRejection()) {
            // A reading the filter distrusts is worse than none - drop the aim at once
            // rather than letting the tolerance keep a bad heading alive.
            consecutiveBadAimReadings++;
            SharedData.getLimelightRawPosition().reset();
            return;
        }

        consecutiveBadAimReadings = 0;

        TagObservation observation = goalFix.observation;
        double xAprilTagRelToCamMM = observation.getTagRelCamXMM();
        double yAprilTagRelToCamMM = observation.getTagRelCamYMM();
        double zAprilTagRelToCamMM = observation.getTagRelCamZMM(); // front back offset from tag
        double distanceFromCamToAprilTag = observation.getGroundRangeMM();

        // Goal offset is fixed relative to the AprilTag - always add in positive X direction
        double adjustedX = xAprilTagRelToCamMM;
        double adjustedZ = zAprilTagRelToCamMM + GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_Z;
        double estimateHeadingFromCamToGoal = Math.atan2(adjustedX, adjustedZ);

        KLog.d("AprilTag_GOAL", () -> String.format(Locale.US,
                "TagPos(x=%.1f, z=%.1f) + Offset -> Adj(x=%.1f, z=%.1f) | AngleToGoal=%.2f deg | Dist=%.1fmm",
                xAprilTagRelToCamMM, zAprilTagRelToCamMM, adjustedX, adjustedZ,
                Math.toDegrees(estimateHeadingFromCamToGoal), distanceFromCamToAprilTag));

        if (!goalFix.verdict.isUsable()) {
            KLog.d("AprilTag_STABILITY", () -> String.format(Locale.US,
                    "Goal tag %d passed every gate but is still warming up (%d/%d clean frames)",
                    goalAprilTagId, filterFor(goalAprilTagId).getConsecutiveGoodReadings(),
                    TagPoseFilter.STABILITY_THRESHOLD));
            return;
        }

        SharedData.setLimelightRawPosition(new LimelightPos(
                distanceFromCamToAprilTag, estimateHeadingFromCamToGoal,
                xAprilTagRelToCamMM, yAprilTagRelToCamMM, zAprilTagRelToCamMM));
    }

    private void logFix(TagFix fix) {
        KLog.d("AprilTag_FIX", () -> String.format(Locale.US,
                "tag %d %s | range=%.0fmm | camRelTag(x=%.1f, y=%.1f, th=%.2f deg) | global(x=%.1f, y=%.1f, th=%.2f deg)",
                fix.observation.getTagId(), fix.verdict,
                fix.observation.getGroundRangeMM(),
                fix.observation.getCamRelTagPos().getX(),
                fix.observation.getCamRelTagPos().getY(),
                Math.toDegrees(fix.observation.getCamRelTagPos().getTheta()),
                fix.globalPos.getX(), fix.globalPos.getY(),
                Math.toDegrees(fix.globalPos.getTheta())));
    }

    private TagPoseFilter filterFor(int tagId) {
        TagPoseFilter filter = filters.get(tagId);
        if (filter == null) {
            filter = new TagPoseFilter(tagId);
            filters.put(tagId, filter);
        }
        return filter;
    }

    // -------------------------------------------------------------------------
    // Read side, for telemetry and for callers deciding whether to trust the pose
    // -------------------------------------------------------------------------

    /** Tag that produced the published field position last update, or NO_TAG. */
    public int getLastFixTagId() { return lastFixTagId; }

    /** True when the last update relocalized off some tag. */
    public boolean hasFix() { return lastFixTagId != NO_TAG; }

    /** Last published field position, or null if the robot has never localized. */
    public Position getGlobalPos() { return globalPos; }

    /** The tag the turret aims off. */
    public int getGoalAprilTagId() { return goalAprilTagId; }

    /** Every tag this action can relocalize from. */
    public AprilTagFieldLayout getFieldLayout() { return fieldLayout; }

    /** Last verdict for one tag, or null if that tag has not been seen yet. */
    public TagPoseFilter.Verdict getVerdictFor(int tagId) {
        TagPoseFilter filter = filters.get(tagId);
        return (filter == null) ? null : filter.getLastVerdict();
    }

    /** Throw away every tag's history, e.g. after the robot is picked up and moved. */
    public void resetFilters() {
        for (TagPoseFilter filter : filters.values()) {
            filter.reset();
        }
        consecutiveBadAimReadings = 0;
        lastFixTagId = NO_TAG;
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }

    // -------------------------------------------------------------------------

    /** One tag's candidate answer to "where is the robot", plus how much to trust it. */
    private static final class TagFix {
        final TagObservation observation;
        final Position globalPos;
        final TagPoseFilter.Verdict verdict;

        TagFix(TagObservation observation, Position globalPos, TagPoseFilter.Verdict verdict) {
            this.observation = observation;
            this.globalPos = globalPos;
            this.verdict = verdict;
        }

        /** Nearer tags give better poses, so this is the tie-break when several are in view. */
        boolean isNearerThan(TagFix other) {
            return other == null
                    || observation.getGroundRangeMM() < other.observation.getGroundRangeMM();
        }
    }
}
