package org.firstinspires.ftc.teamcode.kalipsorobotics.math;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import java.util.Collections;
import java.util.List;
import java.util.Optional;

public class Path {
    private final List<Position> path;
    private int currentSearchWayPointIndex = 0;

    public static final double PATH_ANGLE_TOLERANCE = Math.toRadians(5);
    private double pathAngleTolerance = 0;
    public Path(List<Position> path) {
        this.pathAngleTolerance = PATH_ANGLE_TOLERANCE;
        this.path = Collections.unmodifiableList(path);
    }

//    public Optional<Position> searchFrom(Position currentPosition, double radius) {
//        for (int i = numSegments() - 1; i >= 0; i--) {
//            Segment segment = getSegment(i);
//
//            Optional<Position> result = segment.lineCircleIntersection(currentPosition, radius);
//
//            if (result.isPresent()) {
//                return result;
//            }
//        }
//
//        return Optional.empty();
//    }
//
//    public Optional<Position> searchFrom(Position currentPosition, double radius) {
//        for (int i = numSegments() - 1; i >= 0; i--) {
//            Segment segment = getSegment(i);
//
//            Optional<Position> result = segment.lineCircleIntersection(currentPosition, radius);
//
//            if (result.isPresent()) {
//                Position position = new Position(result.get().getX(), result.get().getY(), segment.getFinish().getTheta());
//                return Optional.of(position);
//            }
//        }
//
//        return Optional.empty();
//    }

    public Optional<Position> lookAhead(Position currentPosition, Optional<Position> lastFollowPosition, double radiusInch) {
        for (int i = currentSearchWayPointIndex; i < numPoints(); i++) {
            Position currentFollowPosition = getPoint(i);

            // has a point to follow
            if (!lastFollowPosition.isPresent()) {
                currentSearchWayPointIndex = i;
                return Optional.of(currentFollowPosition);
            }

            // not within distance
            if (currentPosition.distanceTo(currentFollowPosition) > radiusInch) {
                currentSearchWayPointIndex = i;
                return Optional.of(currentFollowPosition);
            } else {
                // within distance and try to lock angle
                if (Math.abs(currentPosition.getTheta() - currentFollowPosition.getTheta()) > pathAngleTolerance) {
                    currentSearchWayPointIndex = i;
                    KLog.d("purepursaction_debug_follow", "Achieved Point, trying to lock angle | Current Position: " + currentPosition.getPoint()
                    + " Follow Point: " + currentFollowPosition.getPoint());
                    return Optional.of(currentFollowPosition);
                } else {
                    // Waypoint achieved (within distance and angle good) - move to next waypoint on next update cycle
                    currentSearchWayPointIndex = i + 1;
                    KLog.d("purepursaction_debug_follow", "Waypoint achieved, targeting next on next cycle | Achieved: " + currentFollowPosition.getPoint()
                    + " Current Position: " + currentPosition.getPoint());
                    // Continue to check if there's a next waypoint we can target in this iteration
                    // This prevents returning the just-achieved waypoint
                }
            }
        }
        return Optional.empty();
    }

    public Position getPoint(int index) {
        return path.get(index);
    }

    public int numPoints() {
        return path.size();
    }

    public Position getLastPoint() {
        return path.get(path.size() - 1);
    }

    public int numSegments() {
        return numPoints() - 1;
    }

    public Segment getSegment(int index) {
        return new Segment(getPoint(index), getPoint(index + 1));
    }

    public Segment getSegmentIndex(Position position) {
        return new Segment(getPoint(getIndex(position)), getPoint(getIndex(position) + 1));
    }

    public int getIndex(Position position) {
        for (int i = 0; i < path.size(); i++) {

            if (position == path.get(i)) {
                return i;
            }

        }
        return -1;
    }

    public List<Position> getPath() {
        return path;
    }

    public double incrementCurrentSearchWayPointIndex() {
        if ((currentSearchWayPointIndex) < path.size() - 1) {
            currentSearchWayPointIndex++;
        }
        return currentSearchWayPointIndex;
    }

    public double decrementCurrentSearchWayPointIndex() {
        if ((currentSearchWayPointIndex) > 0) {
            currentSearchWayPointIndex--;
        }
        return currentSearchWayPointIndex;
    }


    public double getPathAngleTolerance() {
        return pathAngleTolerance;
    }

    public void setPathAngleTolerance(double pathAngleTolerance) {
        this.pathAngleTolerance = pathAngleTolerance;
    }

    public boolean isFollowingLastPoint() {
        return currentSearchWayPointIndex == path.size()-1;
    }
}
