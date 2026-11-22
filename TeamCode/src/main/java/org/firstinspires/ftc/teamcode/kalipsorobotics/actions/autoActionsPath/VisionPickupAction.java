package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeFullDetector;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.BallCluster;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.BallInformation;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.BallMemory;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.ClusterDetectionAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;

/**
 * V1 vision pickup, one trip: look once, pick the best pile, and drive straight through it with the intake running
 * the whole way. Ends when the intake is full (its motor current says it is packed: the side is raised so it stops
 * taking balls), when the drive reaches the far side of the pile, when nothing is worth picking, or on the timeout.
 * Ends wherever the robot is, intake stopped, ready for the auto to drive to its shoot point.
 *
 * Needs, already running in the OpMode:
 *   - BallVision.start(hardwareMap, BallVision.Detector.TFLITE or COLOR_BLOB, false)  (balls into SharedData)
 *   - odometry
 * No beam breaks: "full" comes from IntakeFullDetector (tune it with IntakeMotorTest).
 *
 * Usage:
 *   VisionPickupAction pickup = new VisionPickupAction(driveTrain, stopper, intake, allianceColor).setTimeoutMs(6000);
 *   pickup.setName("visionPickup");
 *   pickup.setDependentActions(driveToLookPoint);       // stand somewhere the camera can see the balls
 *   auto.addAction(pickup);
 *   goToShoot.setDependentActions(pickup);              // then shoot as usual
 *
 * The drive: AutoToBallsPathingAction to the pile (center-line safe in AUTO), then on past it by the pile's size plus
 * OVERSHOOT_MM, so the whole pile goes through the intake in one pass instead of stopping at each ball.
 *
 * V2 (setMaxStops above 1): after a pass that did not fill the intake it looks again and goes for the next pile,
 * remembering balls it saw earlier (BallMemory). Off by default.
 */
public class VisionPickupAction extends Action {
    public static final String TAG = "VisionPickup";

    /** How many new camera frames the look waits for. */
    public static int LOOK_FRAMES = 3; // TODO-measure: frames for a stable reading
    /** The look gives up waiting after this long and uses what it has. */
    public static double LOOK_TIMEOUT_MS = 500; // TODO-measure
    /** Drive this far past the far edge of the pile so the last ball gets all the way into the robot. */
    public static double OVERSHOOT_MM = 80; // TODO-measure: smallest value where the last ball always gets in
    /** Default number of passes. 1 = V1 (one trip). */
    public static int MAX_STOPS = 1;

    public enum EndReason { NONE, FULL, NOTHING_TO_PICK, MAX_STOPS, TIMEOUT, DROPPED }

    private enum State { LOOK, PICKUP }

    private final DriveTrain driveTrain;
    private final Stopper stopper;
    private final Intake intake;
    private final AllianceColor allianceColor;

    private int maxStops = MAX_STOPS;
    private double timeoutMs = 0; // 0 = no timeout

    private final BallMemory memory = new BallMemory();
    private final IntakeFullDetector fullDetector = new IntakeFullDetector();
    private State state = State.LOOK;
    private long startMs;
    private long lookStartMs;
    private long lastFrameMs;
    private int lookFrames;
    private int stops = 0;
    private BallCluster chosen;
    private AutoToBallsPathingAction pickup;
    private EndReason endReason = EndReason.NONE;

    public VisionPickupAction(DriveTrain driveTrain, Stopper stopper, Intake intake, AllianceColor allianceColor) {
        this.driveTrain = driveTrain;
        this.stopper = stopper;
        this.intake = intake;
        this.allianceColor = allianceColor;
    }

    /** V2: passes per call (default MAX_STOPS = 1). */
    public VisionPickupAction setMaxStops(int maxStops) {
        this.maxStops = maxStops;
        return this;
    }

    /** Give up after this long, wherever the robot is (default: no timeout). Use it to protect the end of the match. */
    public VisionPickupAction setTimeoutMs(double timeoutMs) {
        this.timeoutMs = timeoutMs;
        return this;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        long now = System.currentTimeMillis();
        if (!hasStarted) {
            hasStarted = true;
            startMs = now;
            startLook(now);
        }
        if (timeoutMs > 0 && now - startMs >= timeoutMs) {
            finish(EndReason.TIMEOUT);
            return;
        }

        switch (state) {
            case LOOK:
                if (look(now)) {
                    choose(now);
                }
                break;
            case PICKUP:
                pickup.updateCheckDone();
                IntakeSide side = pickup.getPickupSide();
                if (side != null) {
                    fullDetector.update(side.getCurrentMA(), now);
                    if (fullDetector.isFull()) {
                        side.raise(); // packed: lift it so it stops taking balls, the drive ends here
                        finish(EndReason.FULL);
                        return;
                    }
                }
                if (!pickup.getIsDone()) {
                    return;
                }
                // Picked up, or the planner could not reach them: either way do not choose these balls again.
                for (BallInformation ball : chosen.getBalls()) {
                    memory.forgetNear(ball.toPoint(), BallMemory.MERGE_MM);
                }
                if (stops >= maxStops) {
                    finish(EndReason.MAX_STOPS);
                } else {
                    startLook(now);
                }
                break;
        }
    }

    private void startLook(long now) {
        state = State.LOOK;
        lookStartMs = now;
        lookFrames = 0;
        memory.forgetOld(now);
    }

    /** Adds new frames to the memory. True when the look is over: enough frames with balls seen, or time is up. */
    private boolean look(long now) {
        long frameMs = SharedData.getBallInformationTimeMs();
        if (frameMs >= lookStartMs && frameMs != lastFrameMs) { // a new frame, taken since this look started
            lastFrameMs = frameMs;
            lookFrames++;
            memory.add(SharedData.getBallInformation(), now);
        }
        boolean enough = lookFrames >= LOOK_FRAMES && !memory.getBalls().isEmpty();
        return enough || now - lookStartMs >= LOOK_TIMEOUT_MS;
    }

    private void choose(long now) {
        int room = BallCluster.INTAKE_CAPACITY - fullDetector.estimateBalls();
        Point robot = SharedData.getOdometryWheelIMUPosition().toPoint();
        chosen = ClusterDetectionAction.chooseBest(memory.getBalls(), robot, allianceColor, room);
        if (chosen == null) {
            KLog.d(TAG, () -> "name=" + getName() + " event=nothingToPick remembered=" + memory.getBalls().size() + " room=" + room);
            finish(EndReason.NOTHING_TO_PICK);
            return;
        }
        stops++;
        Point aim = ClusterDetectionAction.aimPointOf(chosen);
        double overshoot = throughPileMm(chosen, aim) + OVERSHOOT_MM;
        SharedData.setTargetClusterXY(aim);
        KLog.d(TAG, () -> "name=" + getName() + " event=chose stop=" + stops + " cluster=" + chosen + " aim=" + aim
                + " overshoot=" + overshoot + " room=" + room);
        pickup = new AutoToBallsPathingAction(driveTrain, stopper, intake, aim, true)
                .setIntakeFromStart(true)
                .setOvershootMm(overshoot);
        pickup.setName(getName() + "_stop" + stops);
        if (stops == 1) {
            fullDetector.reset(now); // the intake starts running now; the latch carries over V2 passes
        }
        state = State.PICKUP;
    }

    /** How far past the aim point the pile reaches: the farthest ball from it. Driving that far takes the whole pile. */
    static double throughPileMm(BallCluster cluster, Point aim) {
        // ponytail: farthest ball in any direction, not along the drive line; over-drives a long pile approached side-on
        double farthest = 0;
        for (BallInformation ball : cluster.getBalls()) {
            farthest = Math.max(farthest, Point.distance(aim, ball.toPoint()));
        }
        return farthest;
    }

    private void finish(EndReason reason) {
        endReason = reason;
        KLog.d(TAG, () -> "name=" + getName() + " event=done reason=" + reason + " stops=" + stops
                + " ballEstimate=" + fullDetector.estimateBalls());
        setIsDone(true);
    }

    /** Done, or dropped by the caller: stop the pickup in progress (that zeroes the wheels and stops the intake). */
    @Override
    public void setIsDone(boolean isDone) {
        super.setIsDone(isDone);
        if (!isDone) {
            return;
        }
        if (endReason == EndReason.NONE) {
            endReason = EndReason.DROPPED;
        }
        if (pickup != null && !pickup.getIsDone()) {
            pickup.setIsDone(true);
        }
    }

    public EndReason getEndReason() {
        return endReason;
    }

    /** Passes started (including one cut short by FULL or a timeout). */
    public int getStops() {
        return stops;
    }

    /** Balls held, estimated from the intake current. */
    public int getBallEstimate() {
        return fullDetector.estimateBalls();
    }
}
