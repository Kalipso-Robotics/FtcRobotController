package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

import java.util.ArrayList;
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
    /**
     * The way the robot's FRONT points at standOff, radians in the FieldConfig frame. When the back
     * intake picks up the ball, the front points away from the ball.
     */
    public final double finalHeadingRad;
    /** True if the last straight stretch goes into the buffer ring around an obstacle. */
    public final boolean enteredRing;
    /**
     * True if the robot stops with its front touching the ball (the intake pulls it in), so the ball can
     * be against an obstacle. False if the ball ends up inside the intake mouth.
     */
    public final boolean pullIn;
    /** True if the BACK intake picks up the ball, false if the front one does. */
    public final boolean useBackIntake;
    public final Reason reason;
    /** One line for the log. */
    public final String why;

    private PickupPlan(List<Point> waypoints, Point standOff, double finalHeadingRad,
                       boolean enteredRing, boolean pullIn, boolean useBackIntake, Reason reason,
                       String why) {
        this.waypoints = waypoints == null ? null : Collections.unmodifiableList(waypoints);
        this.standOff = standOff;
        this.finalHeadingRad = finalHeadingRad;
        this.enteredRing = enteredRing;
        this.pullIn = pullIn;
        this.useBackIntake = useBackIntake;
        this.reason = reason;
        this.why = why;
    }

    static PickupPlan ok(List<Point> waypoints, Point standOff, double headingRad, boolean enteredRing,
                         boolean pullIn, boolean useBackIntake, String why) {
        return new PickupPlan(waypoints, standOff, headingRad, enteredRing, pullIn, useBackIntake, Reason.OK,
                why);
    }

    static PickupPlan fail(Reason reason, String why) {
        return new PickupPlan(null, null, 0, false, false, false, reason, why);
    }

    public boolean isOk() {
        return reason == Reason.OK;
    }

    /**
     * The points for the path follower, one {x, y, headingDeg} row per waypoint. Each waypoint faces the
     * direction of travel, and the last one faces finalHeadingRad so the intake points at the ball.
     * Headings are in degrees, between -180 and 180. Empty if the ball cannot be picked up.
     */
    public List<double[]> followerPoints(Point robot) {
        List<double[]> rows = new ArrayList<>();
        if (waypoints == null) {
            return rows;
        }
        Point previous = robot;
        double headingDeg = Math.toDegrees(finalHeadingRad);
        for (int i = 0; i < waypoints.size(); i++) {
            Point p = waypoints.get(i);
            if (i == waypoints.size() - 1) {
                headingDeg = Math.toDegrees(finalHeadingRad);
            } else if (Point.distance(previous, p) > 1e-6) {
                headingDeg = Math.toDegrees(Math.atan2(p.getY() - previous.getY(), p.getX() - previous.getX()));
            }
            rows.add(new double[]{p.getX(), p.getY(), normalizeDeg(headingDeg)});
            previous = p;
        }
        return rows;
    }

    /** Wraps an angle in degrees into (-180, 180]. */
    private static double normalizeDeg(double deg) {
        double d = deg % 360;
        if (d > 180) {
            d -= 360;
        } else if (d <= -180) {
            d += 360;
        }
        return d;
    }

    @Override
    public String toString() {
        return reason + ": " + why;
    }
}
