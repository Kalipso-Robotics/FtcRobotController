package org.firstinspires.ftc.teamcode.kalipsorobotics.math;


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

    public double distanceTo(Point point2) {
        return Math.hypot(point2.getX() - this.getX(), point2.getY() - this.getY());
    }

    public Point relativeTo(Point point2) {
        return new Point(this.getX() - point2.getX(), this.getY() - point2.getY());
    }

    public static double distance(Point p1, Point p2) {
        return Math.hypot(p2.getX() - p1.getX(), p2.getY() - p1.getY());
    }

    public static double distance(double x1, double y1, double x2, double y2) {
        return Math.hypot(x2 - x1, y2 - y1);
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

    public Point multiplyY(int polarity) {
        return new Point(x, y * polarity);
    }
}
