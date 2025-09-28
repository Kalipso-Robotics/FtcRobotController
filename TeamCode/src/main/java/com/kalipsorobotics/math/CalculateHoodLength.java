package com.kalipsorobotics.math;

public class CalculateHoodLength {
    Position goalPosition;
    public CalculateHoodLength(Position goalPosition) {
        this.goalPosition = goalPosition;
    }
    double launchAngle;
    double gravity = 9.80665;
    double ballExitSpeed; //TODO measure    (meters per second)
    double heightDifference; //TODO measure     (goal height - ball release height)
    public double calculateHoodLengthHighArc(Position robotPosition) {
        launchAngle = Math.atan(Math.pow(ballExitSpeed, 2) +
                Math.sqrt(Math.pow(ballExitSpeed, 4) - gravity*((gravity*Math.pow(calculateHorizontalDistance(robotPosition), 2)) +
                        (2*heightDifference*Math.pow(ballExitSpeed, 2))))/gravity*calculateHorizontalDistance(robotPosition));
        return launchAngle;
    }
    public double calculateHoodLengthFlatArc(Position robotPosition) {
        launchAngle = Math.atan(Math.pow(ballExitSpeed, 2) -
                Math.sqrt(Math.pow(ballExitSpeed, 4) - gravity*((gravity*Math.pow(calculateHorizontalDistance(robotPosition), 2)) +
                        (2*heightDifference*Math.pow(ballExitSpeed, 2))))/gravity*calculateHorizontalDistance(robotPosition));
        return launchAngle;
    }
    private double calculateHorizontalDistance(Position robotPosition) {
        return Math.sqrt(Math.pow(robotPosition.getX() - goalPosition.getX(), 2) + Math.pow(robotPosition.getY() - goalPosition.getY(), 2));
    }
}
