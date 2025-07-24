package com.kalipsorobotics.fresh.math;

public class Position {
    final private double x;
    final private double y;
    final private double theta;

    public Position (double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
    //add point to vector
    public Position add(Velocity velocity) {
        return new Position (
            this.x + velocity.getX(),
            this.y + velocity.getY(),
            this.theta + velocity.getTheta()
        );
    }

    public Point toPoint() {
        return new Point(getX(), getY());
    }

    @Override
    public String toString() {
        return "{" +
                "x=" + x +
                ", y=" + y +
                ", theta=" + theta +
                '}';
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
}