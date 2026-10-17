package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.junit.Test;

public class RobotFootprintTest {
    private static final Point ORIGIN = new Point(0, 0);

    @Test
    public void farAwayBox_doesNotOverlap() {
        assertFalse(RobotFootprint.overlaps(ORIGIN, 0, new double[]{1000, 1000, 1100, 1100}, 0));
    }

    @Test
    public void boxTouchingTheFront_isNotAnOverlap() {
        double f = RobotFootprint.FRONT_MM;
        assertFalse(RobotFootprint.overlaps(ORIGIN, 0, new double[]{f, -50, f + 100, 50}, 0));
    }

    @Test
    public void boxOneMmIntoTheFront_overlaps() {
        double f = RobotFootprint.FRONT_MM;
        assertTrue(RobotFootprint.overlaps(ORIGIN, 0, new double[]{f - 1, -50, f + 100, 50}, 0));
    }

    @Test
    public void margin_growsTheBox() {
        double f = RobotFootprint.FRONT_MM;
        double[] box = {f + 10, -50, f + 100, 50};
        assertFalse(RobotFootprint.overlaps(ORIGIN, 0, box, 0));
        assertTrue(RobotFootprint.overlaps(ORIGIN, 0, box, 20));
    }

    @Test
    public void thinBoxThroughTheMiddle_overlapsEvenThoughNoCornerIsInside() {
        // a 10 mm wall that is much longer than the robot, passing under its middle
        assertTrue(RobotFootprint.overlaps(ORIGIN, 0, new double[]{-5, -1000, 5, 1000}, 0));
    }

    @Test
    public void rotatedBody_sticksOutAtTheCorner() {
        double heading = Math.PI / 4;
        // a point 180 mm ahead and 180 mm to the side is inside the body
        assertTrue(RobotFootprint.overlaps(ORIGIN, heading, new double[]{250, -10, 260, 10}, 0));
        // the body's farthest x is about 374 mm, so a box starting at 380 mm is clear
        assertFalse(RobotFootprint.overlaps(ORIGIN, heading, new double[]{380, -10, 400, 10}, 0));
    }

    @Test
    public void cornersAreOnTheRightSide() {
        double[][] corners = RobotFootprint.corners(new Point(100, 200), Math.PI / 2);
        double maxY = -Double.MAX_VALUE;
        for (double[] c : corners) {
            maxY = Math.max(maxY, c[1]);
        }
        // facing +y, the front tip is FRONT_MM above the center
        assertTrue(Math.abs(maxY - (200 + RobotFootprint.FRONT_MM)) < 1e-6);
    }
}
