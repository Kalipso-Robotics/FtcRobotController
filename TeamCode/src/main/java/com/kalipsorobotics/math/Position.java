package com.kalipsorobotics.math;

import static org.checkerframework.checker.units.UnitsTools.mm;

import com.kalipsorobotics.PID.PidNav;
import com.kalipsorobotics.modules.GoBildaOdoModule;
import com.kalipsorobotics.modules.GoBildaPinpointDriver;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 *
 * EVERYTHING HERE IS RADIANS
 *
 */


public class Position {
    private double x;
    private double y;
    private double theta;

    private double distanceAlongPath;
    private double curvature;
    private double velocity;
    private double acceleration;

    private PidNav pidX = new PidNav(PurePursuitAction.P_XY, 0, 0);
    private PidNav pidY = new PidNav(PurePursuitAction.P_XY, 0, 0);
    private PidNav pidAngle = new PidNav(PurePursuitAction.P_ANGLE, 0, 0);

    public Position (double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public Position (Position position) {
        this(position.x, position.y, position.theta);
    }

    public Position (double x, double y, double theta, double pXY, double pAngle) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.pidX.setP(pXY);
        this.pidY.setP(pXY);
        this.pidAngle.setP(pAngle);
    }

    //add point to vector
    public Position add(Velocity velocity) {
        double theta = this.theta + velocity.getTheta();
        return new Position (
            this.x + velocity.getX(),
            this.y + velocity.getY(),
            MathFunctions.angleWrapRad(theta)
        );
    }

    public Position addPosition(Vector vector) {
        return new Position(this.getX() + vector.getX(), this.getY() + vector.getY(), Math.atan2(vector.getY(),vector.getX()));
    }

//    public Point toPoint() {
//        return new Point(getX(), getY());
//    }

//    public static List<Point> toPointList(List<Position> positions) {
//        List<Point> points = new ArrayList<Point>();
//
//        for(int i=0; i < positions.size(); i++) {
//            points.add(positions.get(i).toPoint());
//        }
//
//        return points;
//    }

    public Position relativeTo(Position other) {
        return new Position(this.getX() - other.getX(), this.getY() - other.getY(), this.getTheta() - other.getTheta());
    }

    public Vector projectOnto(Vector vector) {
        return vector.scale( vector.dot(this)/vector.dot(vector));
    }

    public double distanceTo(Position other) {
        return Math.hypot(other.getX() - this.getX(), other.getY() - this.getY());
    }

    @Override
    public String toString() {
        return  String.format("x=%.2f (%.2f in), y=%.2f (%.2f in), theta=%.4f (%.1f deg)", x, x/25.4, y, y/25.4,
                theta, Math.toDegrees(theta));
    }

    public static Position pose2DtoPosition(Pose2D pose2D) {
        Position position = new Position(pose2D.getX(DistanceUnit.MM), pose2D.getY(DistanceUnit.MM), pose2D.getHeading(AngleUnit.RADIANS));
        return position;
    }

    public String getPoint() {
        return x + ", " + y + ", " + theta;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    public PidNav getPidX() {
        return pidX;
    }

    public PidNav getPidY() {
        return pidY;
    }

    public PidNav getPidAngle() {
        return pidAngle;
    }

    public void addX(double add) {
        x += add;
    }

    public void addY(double add) {
        y += add;
    }

    public void reset(Position position) {
        this.x = position.getX();
        this.y = position.getY();
        this.theta = position.getTheta();
        this.pidX = position.pidX;
        this.pidY = position.pidY;
        this.pidAngle = position.pidAngle;
    }

    public double getDistanceAlongPath() {
        return this.distanceAlongPath;
    }

    public void setDistanceAlongPath(double distanceAlongPath) {
        this.distanceAlongPath = distanceAlongPath;
    }

    public double getCurvature() {
        return this.curvature;
    }

    public void setCurvature(double curvature) {
        this.curvature = curvature;
    }

    public double getVelocity() {
        return velocity;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public void setAcceleration(double acceleration) {
        this.acceleration = acceleration;
    }

    public void setX(double newX) {
        this.x = newX;
    }

    public void setY(double newY) {
        this.y = newY;
    }

}