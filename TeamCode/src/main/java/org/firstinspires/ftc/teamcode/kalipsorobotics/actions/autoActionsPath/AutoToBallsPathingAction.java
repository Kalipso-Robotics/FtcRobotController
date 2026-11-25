package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeFullAction;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.kalipsorobotics.configs.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PathPlanning;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PickupPlan;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.RobotFootprint;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import java.util.List;

/**
 * Drives the robot to a ball (or the middle of a cluster of balls) and stops facing it, so the
 * intake can pick it up.
 *
 * What this action does, in order:
 *   1. Works out where the ball is (from the constructor, or from SharedData).
 *   2. Asks the path planner for a route that goes around the Hive legs and the flowers.
 *      In AUTO the route also stays on our own side of the field (manual rule G402).
 *   3. Hands that route to the path follower, which actually drives the wheels.
 *   4. Turns the intake on when we are close to the ball: the end the planner chose leads, so that side
 *      lowers and runs, and the other side stops and raises. With useFlower, the Flower arm also lowers
 *      once its side is down (Flower piles; the planner is forced to use the flower's side).
 *   5. Finishes when the robot has arrived, and stops the intake (and raises the flower) then or when dropped.
 *
 * If there is no ball to go to, or the planner says the ball cannot be picked up, the action
 * finishes straight away without driving. Then getFailed() is true and getPlan() says why,
 * so the autonomous program does not get stuck waiting for it.
 *
 * All positions are in millimeters, in our own alliance's field frame (the same frame the
 * odometry uses).
 */
public class AutoToBallsPathingAction extends Action {
    public static final String TAG = "Auto_Path";

    // ------------------------------------------------------------------------------------------
    // Settings you can change
    // ------------------------------------------------------------------------------------------

    /**
     * How much empty space the planner keeps around every obstacle, in mm. It stands in for the
     * size of the robot. PLACEHOLDER: set it from the real robot (see RobotFootprint).
     */
    public static double BUFFER_MM = 262.5;

    /**
     * When the robot is this close to its stopping point, in mm, the intake turns on.
     * (If the front has to touch the ball and pull it in, the intake turns on right away.)
     */
    public static double INTAKE_START_DISTANCE_MM = 600; // TODO-measure

    /**
     * The Flower arm only deploys while the robot's center is at least this far (mm) from the Flower's center,
     * so it never drops onto the Flower. Must be smaller than INTAKE_START_DISTANCE_MM or the arm never deploys.
     */
    public static double FLOWER_DEPLOY_CLEARANCE_MM = 350; // TODO-measure

    /** After arriving, the chosen side keeps running this long so the ball gets pulled through, then the action ends. */
    public static double POST_ARRIVAL_INTAKE_MS = 500; // TODO-measure

    /**
     * AUTO: every corner of the robot body stays at least this far short of the center line (FieldConfig.MIDLINE_X,
     * 1828.8 mm), for odometry drift and coasting. Planned paths keep this margin, an overshoot that would eat into it
     * is dropped, and if the robot gets inside it anyway the drive stops at once (G402: never cross in AUTO).
     */
    public static double AUTO_MIDLINE_MARGIN_MM = 75; // TODO-measure: how far the robot coasts after the drive is cut

    /** How long the chosen side may run before arrival (a pull-in pickup turns it on at the start of the drive). */
    public static double INTAKE_RUN_MS = 3000; // TODO-measure

    // ------------------------------------------------------------------------------------------
    // Things we are given or remember
    // ------------------------------------------------------------------------------------------

    /** The wheels. The path follower needs it to drive. */
    private final DriveTrain driveTrain;

    /** True in AUTO, when we must stay on our own side of the field. False in TELEOP. */
    private final boolean auto;

    private final Stopper stopper;

    /** The intake, or null if we only want to drive. */
    private final Intake intake;

    /** The Flower being picked up, or null if this is not a Flower pile. */
    private final Point flower;

    /** True for a Flower pile: only the flower's side may pick up, and the Flower arm lowers too. */
    private boolean useFlower; // cleared when the flower's side cannot reach the pile and the other side picks up instead

    /** The side the planner chose. Set once the route is planned. */
    private IntakeSide pickupSide;

    /** Runs the chosen side. Made once the route is planned, null if we only drive. */
    private IntakeFullAction intakeFullAction;

    /** True once the Flower arm has been lowered (it is raised again when we finish). */
    private boolean flowerLowered = false;

    /** Where the ball is. If the constructor was not given one, we read it from SharedData at the start. */
    private Point target;

    /** The part that actually drives the wheels along the route. Made once the route is known. */
    private AdaptivePurePursuitAction movement;

    /** The planner's answer: the route, where to stop, which way to face, or why it cannot be done. */
    private PickupPlan plan;

    /** True if we ended without driving because there was no ball or the ball cannot be picked up. */
    private boolean failed = false;

    /** True once the intake has been turned on. After that we keep it running until the end. */
    private boolean intakeOn = false;

    /** Running since the robot arrived; null until then. */
    private ElapsedTime arrivedTimer = null;

    /** True: the intake runs for the whole drive (picks up balls on the way), not only near the stop. */
    private boolean intakeFromStart = false;

    /** How far past the stand-off to drive, mm, so the last ball gets all the way in (0 = stop at the stand-off). */
    private double overshootMm = 0;

    // ------------------------------------------------------------------------------------------
    // Constructors (they run once, when you write "new AutoToBallsPathingAction(...)")
    // ------------------------------------------------------------------------------------------

    /**
     * Use this one when something else chooses the ball and puts it in SharedData.
     * Whatever picks the ball just calls SharedData.setTargetClusterXY(...), and this action
     * reads it when it starts. That way this action does not need to know how the ball was chosen.
     *
     * @param driveTrain the wheels
     * @param auto        true in AUTO (stay on our side), false in TELEOP
     * @param intake      the intake action (for example a RunIntake), or null to only drive
     */
    public AutoToBallsPathingAction(DriveTrain driveTrain, Stopper stopper, Intake intake, boolean auto) {
        // Same as the other constructor, with "no ball yet" (null) as the target.
        this(driveTrain, stopper, intake, null, auto);
    }

    /**
     * Use this one when you already know where the ball is.
     *
     * @param driveTrain the wheels
     * @param target      the ball (or cluster center) position, in mm
     * @param auto        true in AUTO (stay on our side), false in TELEOP
     */

    public AutoToBallsPathingAction(DriveTrain driveTrain, Stopper stopper, Intake intake, Point target, boolean auto) {
        this(driveTrain, stopper, intake, target, auto, null);
    }

    /** @param flower the Flower's center for a Flower pile (pick up with the flower's side and lower the Flower arm), else null */
    public AutoToBallsPathingAction(DriveTrain driveTrain, Stopper stopper, Intake intake, Point target, boolean auto,
                                    Point flower) {
        // Keep our own copy of each thing, because update() runs later and needs them.
        this.driveTrain = driveTrain;
        this.stopper = stopper;
        this.intake = intake;
        this.target = target;
        this.auto = auto;
        this.flower = flower;
        this.useFlower = flower != null;
    }

    /** Run the intake for the whole drive, not only from INTAKE_START_DISTANCE_MM. Call before the action starts. */
    public AutoToBallsPathingAction setIntakeFromStart(boolean intakeFromStart) {
        this.intakeFromStart = intakeFromStart;
        return this;
    }

    /** Drive this far past the stand-off (skipped where the robot would not fit). Call before the action starts. */
    public AutoToBallsPathingAction setOvershootMm(double overshootMm) {
        this.overshootMm = overshootMm;
        return this;
    }

    // ------------------------------------------------------------------------------------------
    // Running the action
    // ------------------------------------------------------------------------------------------

    /**
     * Called over and over (many times a second) until the action is done.
     * The first call plans the route. Every call after that keeps driving it.
     */
    @Override
    protected void update() {
        // Already finished (or failed): nothing more to do.
        if (isDone) {
            return;
        }

        // First call only: plan the route. We do it now, not in the constructor, because
        // now is when we know where the robot is.
        if (!hasStarted) {
            hasStarted = true;
            planTheRoute();

            // If planning failed, the action is already marked done, so stop here.
            if (isDone) {
                return;
            }
        }

        if (auto) {
            Position pose = new Position(SharedData.getOdometryWheelIMUPosition());
            if (nearMidline(pose.toPoint(), pose.getTheta())) {
                KLog.e(TAG, "name=" + getName() + " event=midlineStop pose=" + pose + " margin=" + AUTO_MIDLINE_MARGIN_MM);
                setIsDone(true); // zeroes the wheels and stops the intake
                return;
            }
        }

        // Turn the intake on when we are close enough to the ball, then keep it going.
        if (intakeFullAction != null && (intakeOn || closeEnoughToStartIntake())) {
            if (!intakeOn) {
                intakeOn = true;
                startIntake();
            }
            intakeFullAction.updateCheckDone();
            // The Flower refuses to lower until its side is down, so wait for that instead of asking every loop.
            // Only near the stand-off: a pull-in pickup runs the intake for the whole drive, and the arm is not in the footprint.
            if (useFlower && !flowerLowered && distanceToStop() <= INTAKE_START_DISTANCE_MM) {
                double toFlower = Point.distance(currentRobotPoint(), flower);
                if (shouldLowerFlower(false, pickupSide.isDown(), toFlower)) {
                    flowerLowered = intake.getFlower().lower();
                    if (flowerLowered) {
                        KLog.d(TAG, () -> "name=" + getName() + " event=flowerLower reason=pickupFlower");
                    }
                } else if (toFlower < FLOWER_DEPLOY_CLEARANCE_MM) {
                    useFlower = false; // too close to drop the arm without landing on the Flower
                    KLog.e(TAG, "name=" + getName() + " event=flowerSkipped reason=tooClose dist=" + toFlower);
                }
            }
        }

        // Keep driving the route. When the follower has arrived, keep the intake pulling for a moment, then we are done.
        movement.updateCheckDone();
        if (!movement.getIsDone()) {
            return;
        }
        if (intakeFullAction == null) {
            setIsDone(true);
            return;
        }
        if (arrivedTimer == null) {
            arrivedTimer = new ElapsedTime();
            KLog.d(TAG, () -> "name=" + getName() + " event=arrived holdMs=" + POST_ARRIVAL_INTAKE_MS);
        }
        pickupSide.run(IntakeConfig.intakePower); // the timed run may have ended on a long pull-in drive
        if (arrivedTimer.milliseconds() >= POST_ARRIVAL_INTAKE_MS) {
            setIsDone(true);
        }
    }

    /** True when the Flower arm may deploy now: not already down, its side is down, and the robot is clear of the Flower. */
    public static boolean shouldLowerFlower(boolean flowerDown, boolean sideDown, double distToFlowerMM) {
        return !flowerDown && sideDown && distToFlowerMM >= FLOWER_DEPLOY_CLEARANCE_MM;
    }

    /** True if any corner of the body (centered at center, front along headingRad) is within AUTO_MIDLINE_MARGIN_MM of the center line or past it. */
    public static boolean nearMidline(Point center, double headingRad) {
        for (double[] corner : RobotFootprint.corners(center, headingRad)) {
            if (corner[0] > FieldConfig.MIDLINE_X - AUTO_MIDLINE_MARGIN_MM) {
                return true;
            }
        }
        return false;
    }

    /** KLog.e at OpMode init if the clearance would keep the arm from ever deploying. */
    public static void checkFlowerClearance() {
        if (FLOWER_DEPLOY_CLEARANCE_MM >= INTAKE_START_DISTANCE_MM) {
            KLog.e(TAG, "event=badConfig FLOWER_DEPLOY_CLEARANCE_MM=" + FLOWER_DEPLOY_CLEARANCE_MM
                    + " >= INTAKE_START_DISTANCE_MM=" + INTAKE_START_DISTANCE_MM + " arm will never deploy");
        }
    }

    /** The chosen side lowers (its run comes from intakeFullAction); the other side stops and raises. */
    private void startIntake() {
        pickupSide.lower();
        BeamBreakPosition.IntakeSide other = pickupSide.getSide() == BeamBreakPosition.IntakeSide.A
                ? BeamBreakPosition.IntakeSide.B : BeamBreakPosition.IntakeSide.A;
        intake.getSide(other).stop();
        intake.getSide(other).raise();
        KLog.d(TAG, () -> "name=" + getName() + " event=intakeOn side=" + pickupSide.getSide());
    }

    /** Done, or dropped by the caller: stop the intake and raise the Flower arm. */
    @Override
    public void setIsDone(boolean isDone) {
        super.setIsDone(isDone);
        if (!isDone) {
            return;
        }
        if (movement != null && !movement.getIsDone()) {
            movement.setIsDone(true); // dropped mid-drive: zero the wheels instead of leaving the last power on
        }
        if (intakeFullAction != null) {
            intakeFullAction.stopAndSetDone();
        }
        if (flowerLowered) {
            flowerLowered = false;
            intake.getFlower().raise();
            KLog.d(TAG, () -> "name=" + getName() + " event=flowerRaise reason=pickupDone");
        }
    }

    /**
     * Finds the ball, asks the planner for a route, and gets the path follower ready.
     * If any step cannot be done, it calls fail(...), which ends the action.
     */
    private void planTheRoute() {
        // Step 1: which ball are we going to?
        // If we were not given one, use the one the decision code left in SharedData.
        if (target == null) {
            target = SharedData.getTargetClusterXY();
        }
        if (target == null) {
            fail("no target cluster is set");
            return;
        }

        // Step 2: where is the robot right now, and which way does its front point?
        Position pose = new Position(SharedData.getOdometryWheelIMUPosition());
        Point robot = pose.toPoint();

        // Step 3: make a planner for the field.
        // In AUTO it also treats the other alliance's half of the field as blocked. The second
        // number is how far the robot's center must stay from the middle line: half the robot's
        // width, so the whole body stays on our side.
        // In TELEOP, while the Hive is tipping, nothing may drive under it, so that space is blocked too.
        // In AUTO the space under the Hive stays open, so the robot may drive under it even while it tips.
        boolean hiveTipping = !auto && SharedData.isHiveTipping();
        PathPlanning planner;
        if (auto) {
            planner = PathPlanning.forFieldAuto(BUFFER_MM, RobotFootprint.WIDTH_MM / 2 + AUTO_MIDLINE_MARGIN_MM, hiveTipping);
        } else {
            planner = PathPlanning.forField(BUFFER_MM, hiveTipping);
        }

        if (flower != null && intake != null && intake.getFlower().isDown()) {
            flowerLowered = true; // already down: skip the deploy (it is raised again when we finish)
            KLog.d(TAG, () -> "name=" + getName() + " event=flowerAlreadyDown");
        }

        // Step 4: ask the planner how to pick up this ball. We tell it which way the robot faces now,
        // so it can prefer the intake end that needs the smaller turn.
        // It gives back a route, where the robot should stop, which way its front should point there,
        // and which intake (front or back) picks the ball up.
        // A Flower pile can only be picked up by the side the Flower arm is on.
        Boolean forceBack = useFlower ? ModuleConfig.FLOWER_SIDE == BeamBreakPosition.IntakeSide.B : null;
        plan = planner.planPickup(robot, pose.getTheta(), target, forceBack);
        if (forceBack != null && !plan.isOk()) {
            // The flower's side cannot reach this pile: pick it up with whichever side can, without the arm.
            String forcedWhy = plan.why;
            plan = planner.planPickup(robot, pose.getTheta(), target, null);
            useFlower = plan.isOk() && plan.useBackIntake == forceBack;
            KLog.d(TAG, () -> "name=" + getName() + " event=flowerFallback forcedSide=" + (forceBack ? "back" : "front")
                    + " forcedWhy=" + forcedWhy);
        }
        KLog.d(TAG, () -> "name=" + getName() + " event=plan robot=" + robot + " target=" + target
                + " auto=" + auto + " useFlower=" + useFlower + " plan=" + plan);

        // The planner can say no (for example: the ball is in a corner we cannot reach).
        if (!plan.isOk()) {
            fail(plan.why);
            return;
        }

        // Step 5: give the route to the path follower, one point at a time.
        movement = new AdaptivePurePursuitAction(driveTrain);
        movement.setName(getName() + "_move");

        // Plan the move from where the robot is NOW, not from the end of whatever move came before.
        movement.setPlanStartFrom(null);

        // Each row is {x, y, heading in degrees}. The heading is the way the robot's FRONT should point
        // at that point: along the direction it is travelling, and at the last point whatever the plan
        // says (toward the ball if the front picks it up, away from it if the back does).
        List<double[]> points = plan.followerPoints(robot);
        if (!points.isEmpty()) {
            Point end = planner.overshoot(plan, overshootMm); // the stand-off when overshootMm is 0
            if (auto && nearMidline(end, plan.finalHeadingRad)) {
                end = plan.standOff; // the overshoot would reach the center line margin
            }
            points.get(points.size() - 1)[0] = end.getX();
            points.get(points.size() - 1)[1] = end.getY();
        }
        for (double[] point : points) {
            double x = point[0];
            double y = point[1];
            double headingDegrees = point[2];
            movement.addPoint(x, y, headingDegrees);
        }

        // Step 6: the intake on the end the planner chose (it runs from INTAKE_START_DISTANCE_MM on).
        if (intake != null) {
            pickupSide = intake.getSide(plan.useBackIntake ? BeamBreakPosition.IntakeSide.B : BeamBreakPosition.IntakeSide.A);
            // From the start, the run must last the whole drive; setIsDone stops it when we finish.
            double runMs = intakeFromStart ? Double.MAX_VALUE : INTAKE_RUN_MS;
            intakeFullAction = new IntakeFullAction(stopper, pickupSide, runMs, IntakeConfig.intakePower);
            intakeFullAction.setName(getName() + "_intake");
        }
    }

    /** Where the robot is right now, from odometry, as a point on the field (mm). */
    private Point currentRobotPoint() {
        Position pose = new Position(SharedData.getOdometryWheelIMUPosition());
        return pose.toPoint();
    }

    /**
     * True when it is time to turn the intake on:
     *   - at once, if the robot will stop with its tip touching the ball (the intake has to be
     *     running to pull the ball in), or
     *   - once the robot is within INTAKE_START_DISTANCE_MM of its stopping point.
     */
    private boolean closeEnoughToStartIntake() {
        if (plan.pullIn || intakeFromStart) {
            return true;
        }
        return distanceToStop() <= INTAKE_START_DISTANCE_MM;
    }

    /** Straight-line distance from the robot to its stopping point, mm. */
    private double distanceToStop() {
        return Point.distance(currentRobotPoint(), plan.standOff);
    }

    /** Ends the action right now without driving, and remembers that it failed (and why). */
    private void fail(String reason) {
        failed = true;
        isDone = true;
        KLog.d(TAG, () -> "name=" + getName() + " event=failed reason=" + reason);
    }

    // ------------------------------------------------------------------------------------------
    // Ways for other code to look at the result
    // ------------------------------------------------------------------------------------------

    /** True if the action ended without driving: no ball was set, or the ball cannot be picked up. */
    public boolean getFailed() {
        return failed;
    }

    /**
     * The planner's answer, including the reason if it said no. It is null until the action has
     * started, and also null if the action failed because no ball was set.
     */
    public PickupPlan getPlan() {
        return plan;
    }

    /** The intake side picking up, or null until the route is planned (or if there is no intake). */
    public IntakeSide getPickupSide() {
        return pickupSide;
    }

    public AdaptivePurePursuitAction getMovement() {
        return movement;
    }
}
