package org.firstinspires.ftc.teamcode.kalipsorobotics.math;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Velocity {
    private double x;
    private double y;
    private double theta;

    public Velocity(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public Velocity(Velocity v) {
        this(v.getX(), v.getY(), v.getTheta());
    }

//add vector to vector
//    public Velocity add (Velocity velocity) {
//        return new Velocity(
//          this.x + velocity.x,
//          this.y + velocity.y,
//          this.theta + velocity.y
//        );
//    }

    public static Velocity pose2DtoVelocity(Pose2D pose2D) {
        Velocity velocity = new Velocity(pose2D.getX(DistanceUnit.MM), pose2D.getY(DistanceUnit.MM), pose2D.getHeading(AngleUnit.RADIANS));
        return velocity;
    }

    public Velocity divide(double denominator) {
        return new Velocity(this.x / denominator, this.y / denominator, this.theta / denominator);
    }
    public boolean isWithinThreshhold(double thresholdX, double threshholdY, double threshholdTheta) {
        return (Math.abs(this.x) < thresholdX && Math.abs(this.y) < threshholdY && Math.abs(this.theta) < threshholdTheta);
    }
    @Override
    public String toString() {
        return String.format("(x=%.2f, y=%.2f, theta=%.6f)", x, y, theta);
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

    public void reset(Velocity velocity) {
        this.x = velocity.getX();
        this.y = velocity.getY();
        this.theta = velocity.getTheta();
    }
}

