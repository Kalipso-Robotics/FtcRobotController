package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

/**
 * The robot's body as a rectangle on the floor, for checking whether it would touch an obstacle.
 * The robot center is the origin and the body's +x is the direction its FRONT faces. Both ends have
 * an intake (front and back), and both share one pool of at most 4 balls.
 * All units are mm in the FieldConfig frame.
 *
 * PLACEHOLDER numbers (18 in square robot). Measure the real robot and update.
 */
public class RobotFootprint {
    /** Full width, side to side. */
    public static double WIDTH_MM = 457;
    /** Center to the front tip, including the intake. */
    public static double FRONT_MM = 300;
    /** Center to the back. */
    public static double BACK_MM = 229;
    /** Front intake: center to where the ball sits when it is held. Must be at most FRONT_MM. */
    public static double FRONT_GRAB_MM = 260;
    /** Back intake: center to where the ball sits when it is held. Must be at most BACK_MM. PLACEHOLDER. */
    public static double BACK_GRAB_MM = 189;
    /** A ball is 71 mm across (game manual). */
    public static final double BALL_RADIUS_MM = 35.5;

    /** Center to the held ball for the front intake (back = false) or the back intake (back = true). */
    public static double grabMm(boolean back) {
        return back ? BACK_GRAB_MM : FRONT_GRAB_MM;
    }

    /**
     * Center to the ball center when the leading tip just touches the ball. The intake can then pull
     * the ball in, so the ball may sit right against a Hive leg.
     */
    public static double pullInMm(boolean back) {
        return (back ? BACK_MM : FRONT_MM) + BALL_RADIUS_MM;
    }

    /** The four body corners for a robot centered at c facing headingRad, as {x, y} pairs. */
    public static double[][] corners(Point c, double headingRad) {
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        double half = WIDTH_MM / 2;
        double[][] local = {{FRONT_MM, half}, {FRONT_MM, -half}, {-BACK_MM, -half}, {-BACK_MM, half}};
        double[][] result = new double[4][2];
        for (int i = 0; i < 4; i++) {
            result[i][0] = c.getX() + local[i][0] * cos - local[i][1] * sin;
            result[i][1] = c.getY() + local[i][0] * sin + local[i][1] * cos;
        }
        return result;
    }

    /**
     * True if the body overlaps the box. Touching edges is not an overlap.
     * Separating axis test: the rectangle and the box only miss each other if some axis (a side
     * direction of either shape) shows a gap between them.
     *
     * @param box      {minX, minY, maxX, maxY}
     * @param marginMM the box is treated as this much bigger on every side
     */
    public static boolean overlaps(Point c, double headingRad, double[] box, double marginMM) {
        double[][] body = corners(c, headingRad);
        double[][] boxCorners = {
                {box[0] - marginMM, box[1] - marginMM}, {box[2] + marginMM, box[1] - marginMM},
                {box[2] + marginMM, box[3] + marginMM}, {box[0] - marginMM, box[3] + marginMM}};

        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        double[][] axes = {{1, 0}, {0, 1}, {cos, sin}, {-sin, cos}};
        for (double[] axis : axes) {
            double[] a = project(body, axis);
            double[] b = project(boxCorners, axis);
            if (a[1] <= b[0] || b[1] <= a[0]) {
                return false;
            }
        }
        return true;
    }

    /** Smallest and largest position of the points along the axis. */
    private static double[] project(double[][] points, double[] axis) {
        double min = Double.MAX_VALUE;
        double max = -Double.MAX_VALUE;
        for (double[] p : points) {
            double d = p[0] * axis[0] + p[1] * axis[1];
            min = Math.min(min, d);
            max = Math.max(max, d);
        }
        return new double[]{min, max};
    }
}
