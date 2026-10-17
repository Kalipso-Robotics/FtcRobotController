package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PathPlanning;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PickupPlan;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.RobotFootprint;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * Drives the robot to a ball (or the middle of a cluster of balls) and stops facing it, so the
 * intake can pick it up.
 *
 * What this action does, in order:
 *   1. Works out where the ball is (from the constructor, or from SharedData).
 *   2. Asks the path planner for a route that goes around the Hive legs and the flowers.
 *      In AUTO the route also stays on our own side of the field (manual rule G402).
 *   3. Hands that route to the path follower, which actually drives the wheels.
 *   4. Turns the intake on when we are close to the ball. Both ends run together, so the action does not
 *      pick one. The planner still decides which end leads into the ball, to set the robot's heading.
 *   5. Finishes when the robot has arrived.
 *
 * If there is no ball to go to, or the planner says the ball cannot be picked up, the action
 * finishes straight away without driving. Then getFailed() is true and getPlan() says why,
 * so the autonomous program does not get stuck waiting for it.
 *
 * All positions are in millimeters, in our own alliance's field frame (the same frame the
 * odometry uses).
 */
public class AutoToBallsPathingAction extends Action {

    // ------------------------------------------------------------------------------------------
    // Settings you can change
    // ------------------------------------------------------------------------------------------

    /**
     * How much empty space the planner keeps around every obstacle, in mm. It stands in for the
     * size of the robot. PLACEHOLDER: set it from the real robot (see RobotFootprint).
     */
    public static double BUFFER_MM = 350;

    /**
     * When the robot is this close to its stopping point, in mm, the intake turns on.
     * (If the front has to touch the ball and pull it in, the intake turns on right away.)
     */
    public static double INTAKE_START_DISTANCE_MM = 600;

    // ------------------------------------------------------------------------------------------
    // Things we are given or remember
    // ------------------------------------------------------------------------------------------

    /** The wheels. The path follower needs it to drive. */
    private final DriveTrain driveTrain;

    /** True in AUTO, when we must stay on our own side of the field. False in TELEOP. */
    private final boolean auto;

    /** The intake action (front and back run together), or null if we only want to drive. */
    private final Action intake;

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
    public AutoToBallsPathingAction(DriveTrain driveTrain, boolean auto, Action intake) {
        // Same as the other constructor, with "no ball yet" (null) as the target.
        this(driveTrain, null, auto, intake);
    }

    /**
     * Use this one when you already know where the ball is.
     *
     * @param driveTrain the wheels
     * @param target      the ball (or cluster center) position, in mm
     * @param auto        true in AUTO (stay on our side), false in TELEOP
     * @param intake      the intake action, or null to only drive
     */
    public AutoToBallsPathingAction(DriveTrain driveTrain, Point target, boolean auto, Action intake) {
        // Keep our own copy of each thing, because update() runs later and needs them.
        this.driveTrain = driveTrain;
        this.target = target;
        this.auto = auto;
        this.intake = intake;
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

        // Turn the intake on when we are close enough to the ball, then keep it going.
        if (intake != null && (intakeOn || closeEnoughToStartIntake())) {
            intakeOn = true;
            intake.updateCheckDone();
        }

        // Keep driving the route. When the follower says it has arrived, we are done too.
        movement.updateCheckDone();
        if (movement.getIsDone()) {
            isDone = true;
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
        PathPlanning planner;
        if (auto) {
            planner = PathPlanning.forFieldAuto(BUFFER_MM, RobotFootprint.WIDTH_MM / 2);
        } else {
            planner = PathPlanning.forField(BUFFER_MM);
        }

        // Step 4: ask the planner how to pick up this ball. We tell it which way the robot faces now,
        // so it can prefer the intake end that needs the smaller turn.
        // It gives back a route, where the robot should stop, which way its front should point there,
        // and which intake (front or back) picks the ball up.
        plan = planner.planPickup(robot, pose.getTheta(), target);
        KLog.d("AutoToBalls", () -> "robot " + robot + " target " + target
                + (auto ? " (AUTO)" : "") + " -> " + plan);

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
        for (double[] point : plan.followerPoints(robot)) {
            double x = point[0];
            double y = point[1];
            double headingDegrees = point[2];
            movement.addPoint(x, y, headingDegrees);
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
        if (plan.pullIn) {
            return true;
        }
        double distanceToStop = Point.distance(currentRobotPoint(), plan.standOff);
        return distanceToStop <= INTAKE_START_DISTANCE_MM;
    }

    /** Ends the action right now without driving, and remembers that it failed (and why). */
    private void fail(String reason) {
        failed = true;
        isDone = true;
        KLog.d("AutoToBalls", () -> "failed: " + reason);
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
}
