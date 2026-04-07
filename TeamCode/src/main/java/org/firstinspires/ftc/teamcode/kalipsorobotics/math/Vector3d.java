package org.firstinspires.ftc.teamcode.kalipsorobotics.math;

public class Vector3d {
    private double x, y, z;

    // Constructor to create a new vector
    public Vector3d(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    // Multiplies the vector by a single number (Scalar)
    // Useful for Step 5: worldRay.multiply(t)
    public Vector3d multiply(double scalar) {
        return new Vector3d(x * scalar, y * scalar, z * scalar);
    }

    // Adds another vector to this one
    // Useful for adding the Camera's Position to the Ray
    public Vector3d add(Vector3d other) {
        return new Vector3d(this.x + other.x, this.y + other.y, this.z + other.z);
    }

    // Optional: Returns the length of the vector
    public double magnitude() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    // Optional: Forces the vector to have a length of 1
    public Vector3d normalize() {
        double mag = magnitude();
        if (mag == 0) return new Vector3d(0, 0, 0);
        return multiply(1.0 / mag);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }
}