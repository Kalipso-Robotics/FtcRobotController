package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

import java.util.Collections;
import java.util.List;

/**
 * How to pick up one ball: drive through the waypoints to the stand-off point, facing
 * finalHeadingRad, and the intake then reaches the ball. waypoints is null when the ball cannot be
 * picked up, and reason says why.
 */
public class PickupPlan {
    public enum Reason {
        OK,
        /** The ball is off the field. */
        OFF_FIELD,
        /** AUTO: the ball is on the opposing alliance's side (manual G402). */
        OPPOSING_SIDE,
        /** The ball is in a deadzone box (inside corners of the Hive legs). */
        IN_DEADZONE,
        /** The ball is inside a leg or flower. */
        INSIDE_OBSTACLE,
        /** No approach direction keeps the robot body clear of obstacles. */
        NO_APPROACH,
        /** Some approach direction is clear, but the robot cannot get there. */
        NO_PATH
    }

    /** Waypoints after the robot, ending at standOff. Null if the ball cannot be picked up. */
    public final List<Point> waypoints;
    /** Where the robot center ends up. */
    public final Point standOff;
    /** The way the robot faces at standOff, radians in the FieldConfig frame. */
    public final double finalHeadingRad;
    /** True if the last straight stretch goes into the buffer ring around an obstacle. */
    public final boolean enteredRing;
    /**
     * True if the robot stops with its front touching the ball (the intake pulls it in), so the ball can
     * be against an obstacle. False if the ball ends up inside the intake mouth.
     */
    public final boolean pullIn;
    public final Reason reason;
    /** One line for the log. */
    public final String why;

    private PickupPlan(List<Point> waypoints, Point standOff, double finalHeadingRad,
                       boolean enteredRing, boolean pullIn, Reason reason, String why) {
        this.waypoints = waypoints == null ? null : Collections.unmodifiableList(waypoints);
        this.standOff = standOff;
        this.finalHeadingRad = finalHeadingRad;
        this.enteredRing = enteredRing;
        this.pullIn = pullIn;
        this.reason = reason;
        this.why = why;
    }

    static PickupPlan ok(List<Point> waypoints, Point standOff, double headingRad, boolean enteredRing,
                         boolean pullIn, String why) {
        return new PickupPlan(waypoints, standOff, headingRad, enteredRing, pullIn, Reason.OK, why);
    }

    static PickupPlan fail(Reason reason, String why) {
        return new PickupPlan(null, null, 0, false, false, reason, why);
    }

    public boolean isOk() {
        return reason == Reason.OK;
    }

    @Override
    public String toString() {
        return reason + ": " + why;
    }
}
