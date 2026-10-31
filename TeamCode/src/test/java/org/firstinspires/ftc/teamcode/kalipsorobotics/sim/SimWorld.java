package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import java.util.ArrayList;
import java.util.List;

/**
 * The robot's body and the field, as the sim sees them. The fake hardware sets motor powers here
 * and reads sensors from here.
 *
 * Two versions: RapierWorld (the real one: walls, obstacles, Rapier physics) and KinematicWorld (an
 * open field with the same drivetrain model, kept as a cross-check).
 *
 * Frame: the same as Odometry. Field x, y in mm, theta in radians. Robot frame: x forward, y left.
 */
public interface SimWorld extends SimClock.TimeListener, AutoCloseable {

    void setPose(double xMm, double yMm, double thetaRad);

    double getX();

    double getY();

    double getTheta();

    /** The power the code passed to setPower(). The motors are named as in the robot configuration. */
    void setMotorPower(String motorName, double power);

    void setBrake(String motorName, boolean brakeWhenZeroPower);

    /** Raw ticks of the encoder plugged into a motor port. The dead-wheel pods are on bLeft, bRight and fRight. */
    double rawEncoderTicks(String motorName);

    /** IMU yaw in degrees, the way Odometry reads it: heading = -yaw. */
    double rawYawDegrees();

    /**
     * A value the ball hardware currently has, read again every time the world catches up. Names: "intake" (motor power),
     * "stopper" and "hood" (servo positions 0-1), "flywheelRps" (shooter speed), "turretRad" (turret angle from straight ahead).
     */
    default void setActuator(String name, java.util.function.DoubleSupplier value) {
    }

    /** True if the beam at this position is blocked by a held ball. 0 = A_OUTER, 1 = A_INNER, 2 = B_INNER, 3 = B_OUTER. */
    default boolean beamBlocked(int position) {
        return false;
    }

    /** How many balls the robot holds. */
    default int heldBalls() {
        return 0;
    }

    /** What each ball is, as {id, type (0 pollen, 1 red nectar, 2 blue nectar), radius in mm}. Empty if the world has no balls. */
    default List<double[]> ballInfo() {
        return new ArrayList<>();
    }

    /** Balls that moved since the last call (every ball the first time), as {id, x, y, z} in mm. */
    default List<double[]> ballsMoved() {
        return new ArrayList<>();
    }

    /** Every ball right now, as {id, type, x, y, z, held (1 or 0)} in mm. Empty if the world has no balls. */
    default List<double[]> ballPositions() {
        return new ArrayList<>();
    }

    /** The red and blue Hive tilt in radians (positive lifts the +y Cell). */
    default double[] hiveAngles() {
        return new double[]{0, 0};
    }

    /** Tells the world the match has started, so the partner robot's path clock starts. */
    default void partnerStart(long nanos) {
    }

    /** The partner robot R2's pose now as {x, y, theta}, or null when there is none. */
    default double[] partnerPose() {
        return null;
    }

    /** The partner's geometry as "name front back width", or null when there is none. */
    default String partnerInfo() {
        return null;
    }

    /** Things that happened since the last call, as "seconds|text" (virtual seconds), e.g. "7.412|robot touched hive-leg-low". */
    default List<String> drainEvents() {
        return new ArrayList<>();
    }

    /** Problems with the setup that make the run less trustworthy, e.g. a start pose inside a wall. */
    default List<String> warnings() {
        return new ArrayList<>();
    }

    @Override
    default void close() {
    }
}
