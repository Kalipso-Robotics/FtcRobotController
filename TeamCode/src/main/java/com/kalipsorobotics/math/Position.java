package com.kalipsorobotics.math;


import com.kalipsorobotics.PID.PidNav;
import com.kalipsorobotics.modules.GoBildaOdoModule;
import com.kalipsorobotics.modules.GoBildaPinpointDriver;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.concurrent.locks.ReentrantReadWriteLock;

/**
 *
 * EVERYTHING HERE IS RADIANS
 *
 */


public class Position {
    private final ReentrantReadWriteLock lock = new ReentrantReadWriteLock();

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
    private PidNav pidAngleAdaptive = new PidNav(1.3 / Math.toRadians(90), 0.001, 0);

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

    public Point toPoint() {
        return new Point(getX(), getY());
    }

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
        lock.readLock().lock();
        try {
            return x + ", " + y + ", " + theta;
        } finally {
            lock.readLock().unlock();
        }
    }

    public double getX() {
        lock.readLock().lock();
        try {
            return x;
        } finally {
            lock.readLock().unlock();
        }
    }

    public double getY() {
        lock.readLock().lock();
        try {
            return y;
        } finally {
            lock.readLock().unlock();
        }
    }

    public double getTheta() {
        lock.readLock().lock();
        try {
            return theta;
        } finally {
            lock.readLock().unlock();
        }
    }

    public PidNav getPidX() {
        lock.readLock().lock();
        try {
            return pidX;
        } finally {
            lock.readLock().unlock();
        }
    }

    public PidNav getPidY() {
        lock.readLock().lock();
        try {
            return pidY;
        } finally {
            lock.readLock().unlock();
        }
    }

    public PidNav getPidAngle() {
        lock.readLock().lock();
        try {
            return pidAngle;
        } finally {
            lock.readLock().unlock();
        }
    }

    public PidNav getPidAngleAdaptive() {
        return pidAngleAdaptive;
    }

    public void addX(double add) {
        x += add;
    }

    public void addY(double add) {
        y += add;
    }

    public void reset(Position position) {
        lock.writeLock().lock();
        try {
            this.x = position.getX();
            this.y = position.getY();
            this.theta = position.getTheta();
            this.pidX = position.pidX;
            this.pidY = position.pidY;
            this.pidAngle = position.pidAngle;
        } finally {
            lock.writeLock().unlock();
        }
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

    /**
     * Transforms this position from a local coordinate frame to a global coordinate frame.
     * The local frame is defined by an origin position with its own location and orientation.
     *
     * @param originX     X position of the local frame's origin in global coordinates
     * @param originY     Y position of the local frame's origin in global coordinates
     * @param originTheta Rotation of the local frame relative to global frame (radians, +CCW)
     * @return A new Position in global coordinates
     */
    public Position toGlobal(double originX, double originY, double originTheta) {
        double cos = Math.cos(originTheta);
        double sin = Math.sin(originTheta);

        double globalX = originX + (this.x * cos - this.y * sin);
        double globalY = originY + (this.x * sin + this.y * cos);
        double globalTheta = MathFunctions.angleWrapRad(this.theta + originTheta);

        return new Position(globalX, globalY, globalTheta);
    }

    /**
     * Transforms this position from a local coordinate frame to a global coordinate frame.
     * The local frame is defined by the given origin position.
     *
     * @param origin The origin of the local frame expressed in global coordinates
     * @return A new Position in global coordinates
     */
    public Position toGlobal(Position origin) {
        return toGlobal(origin.getX(), origin.getY(), origin.getTheta());
    }

}