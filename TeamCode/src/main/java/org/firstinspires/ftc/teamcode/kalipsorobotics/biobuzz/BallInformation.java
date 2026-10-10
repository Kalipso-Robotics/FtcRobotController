package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

import java.util.Locale;
import java.util.Objects;

/**
 * A detected ball on the field.
 * x and y are in millimeters, field frame (same as FieldConfig: garden corner is (0,0)).
 * Convert robot-relative vision coordinates to the field frame before creating one.
 */
public class BallInformation {
    public enum Type {
        BLUE,
        RED,
        YELLOW,
        UNKNOWN
    }

    public final double x;
    public final double y;
    public final Type type;
    /**
     * Detector confidence in [0, 1]; 1 when the source has none.
     */
    public final float confidence;
    /**
     * Robot-to-ball distance (mm) when the frame was captured; NaN when unknown.
     */
    public final double distanceMM;

    public BallInformation(double x, double y, Type type, float confidence, double distanceMM) {
        if (!Double.isFinite(x) || !Double.isFinite(y)) {
            throw new IllegalArgumentException("Ball coordinates must be finite: (" + x + ", " + y + ")");
        }
        this.x = x;
        this.y = y;
        this.type = Objects.requireNonNull(type, "type");
        this.confidence = confidence;
        this.distanceMM = distanceMM;
    }

    public BallInformation(double x, double y, Type type) {
        this(x, y, type, 1f, Double.NaN);
    }


    public BallInformation(Point point, Type type, float confidence, double distanceMM) {
        this(point.getX(), point.getY(), type, confidence, distanceMM);
    }

    public BallInformation(Point point, Type type) {
        this(point.getX(), point.getY(), type);
    }

    public Point toPoint() {
        return new Point(x, y);
    }

    public double distanceTo(BallInformation other) {
        return Math.hypot(this.x - other.x, this.y - other.y);
    }

    public double distanceTo(Point point) {
        return Math.hypot(this.x - point.getX(), this.y - point.getY());
    }

    public double distanceTo(double targetX, double targetY) {
        return Math.hypot(this.x - targetX, this.y - targetY);
    }

    /**
     * True if other is the same type and within toleranceMM, i.e. likely a duplicate detection of the same ball.
     */
    public boolean isSameBall(BallInformation other, double toleranceMM) {
        return this.type == other.type && distanceTo(other) <= toleranceMM;
    }

    @Override
    public String toString() {
        return String.format(Locale.US, "%s at (%.2f, %.2f)", type, x, y);
    }
}
