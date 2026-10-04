package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import java.util.ArrayList;
import java.util.List;

/**
 * Drives to a target point (e.g. a ball cluster centroid) by routing through a loop of waypoints.
 * The path is built when the action starts, since that's when the robot's position is known:
 * enter the loop at the waypoint closest to the robot, leave it at the waypoint closest to the
 * target, and go around whichever direction is shorter.
 */
public class AutoToBallsPathingAction extends Action {

    // Ordered loop: each waypoint connects to its neighbors, and the last connects back to the first.
    List<Point> waypoints = List.of(
            new Point(913,913),
            new Point (1523, 913),
            new Point (1523, 1523),
            new Point (1523, 2132),
            new Point (1523, 2741),
            new Point (913, 2741),
            new Point (913, 2132),
            new Point (913, 1523));

    private final DriveTrain driveTrain;
    private final Point target;
    private AdaptivePurePursuitAction movement;

    public AutoToBallsPathingAction(DriveTrain driveTrain, Point clusterCentroid) {
        this.driveTrain = driveTrain;
        this.target = clusterCentroid;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }

        if (!hasStarted) {
            hasStarted = true;
            movement = buildMovement();
        }

        movement.updateCheckDone();
        if (movement.getIsDone()) {
            isDone = true;
        }
    }

    private AdaptivePurePursuitAction buildMovement() {
        Position robot = new Position(SharedData.getOdometryWheelIMUPosition());
        List<Point> route = planRoute(robot.toPoint());

        AdaptivePurePursuitAction move = new AdaptivePurePursuitAction(driveTrain);
        move.setName(getName() + "_move");
        // Path is planned from the live position, not from the end of whatever move was built before this one
        move.setPlanStartFrom(null);

        // Face the direction of travel on each leg so the robot arrives facing the balls
        Point prev = robot.toPoint();
        for (Point p : route) {
            double headingDeg = Math.toDegrees(Math.atan2(p.getY() - prev.getY(), p.getX() - prev.getX()));
            move.addPoint(p.getX(), p.getY(), headingDeg);
            prev = p;
        }

        KLog.d("AutoToBalls", () -> "route from " + robot.toPoint() + ": " + route);
        return move;
    }

    /** Waypoints to drive through (in order), ending with the target itself. */
    private List<Point> planRoute(Point robot) {
        int n = waypoints.size();
        int start = findClosestWaypointIndex(robot);
        int end = findClosestWaypointIndex(target);

        // Go whichever way around the loop is shorter
        double forwardDist = loopDistance(start, end, 1);
        double backwardDist = loopDistance(start, end, -1);
        int step = forwardDist <= backwardDist ? 1 : -1;

        List<Point> route = new ArrayList<>();
        int i = start;
        route.add(waypoints.get(i));
        while (i != end) {
            i = Math.floorMod(i + step, n);
            route.add(waypoints.get(i));
        }
        route.add(target);
        return route;
    }

    /** Distance walking around the loop from waypoint {@code from} to {@code to}, stepping by {@code step} (+1 or -1). */
    private double loopDistance(int from, int to, int step) {
        int n = waypoints.size();
        double total = 0;
        int i = from;
        while (i != to) {
            int next = Math.floorMod(i + step, n);
            total += dist(waypoints.get(i), waypoints.get(next));
            i = next;
        }
        return total;
    }

    private int findClosestWaypointIndex(Point from) {
        int best = 0;
        double bestDist = Double.MAX_VALUE;

        for (int i = 0; i < waypoints.size(); i++) {
            double d = dist(from, waypoints.get(i));

            if (d < bestDist) {
                bestDist = d;
                best = i;
            }
        }

        return best;
    }

    private double dist(Point a, Point b) {
        return Math.hypot(b.getX() - a.getX(), b.getY() - a.getY());
    }

}
