package com.kalipsorobotics.math;


public class Point {

    final private double x;
    final private double y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public String toString() {
        return "Point{" +
                "x=" + x +
                ", y=" + y +
                '}';
    }

    public double dot(Point other) {
        return (this.getX() * other.getX()) + (this.getY() * other.getY());
    }

    public double distanceTo(Point other) {
        return Math.hypot(other.getX() - this.getX(), other.getY() - this.getY());
    }

    public Point relativeTo(Point other) {
        return new Point(this.getX() - other.getX(), this.getY() - other.getY());
    }


    public Point add(Vector vector) {
        return new Point(this.getX() + vector.getX(), this.getY() + vector.getY());
    }
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

}
