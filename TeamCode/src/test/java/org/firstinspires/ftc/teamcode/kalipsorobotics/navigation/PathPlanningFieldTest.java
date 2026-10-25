package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.junit.Test;

import java.util.List;

/** Checks the real field layout from FieldConfig: two Hive legs with a gap the robot can use. */
public class PathPlanningFieldTest {
    private static final double BUFFER_MM = 350;
    private static final double CENTER = 3657.6 / 2;

    private final PathPlanning planner = PathPlanning.forField(BUFFER_MM);

    @Test
    public void betweenTheLegs_alongY_isDirect() {
        // Opposite Y sides of the frame, down the middle of the gap between the legs.
        List<Point> path = planner.plan(new Point(CENTER, 700), new Point(CENTER, 3000));
        assertNotNull(path);
        assertEquals(1, path.size());
    }

    @Test
    public void acrossTheLegs_alongX_detours() {
        // Opposite X sides at the frame's center height: the legs block a straight line.
        List<Point> path = planner.plan(new Point(300, CENTER), new Point(3300, CENTER));
        assertNotNull(path);
        assertTrue(path.size() > 1);
    }

    @Test
    public void legsAreInsideTheFrameFootprint() {
        for (double[] leg : new double[][]{FieldConfig.HIVE_LEG_LOW_X, FieldConfig.HIVE_LEG_HIGH_X}) {
            assertTrue(leg[0] >= FieldConfig.HIVE_MIN_X && leg[2] <= FieldConfig.HIVE_MAX_X);
            assertTrue(leg[1] >= FieldConfig.HIVE_MIN_Y && leg[3] <= FieldConfig.HIVE_MAX_Y);
        }
        // there is a gap between the legs
        assertTrue(FieldConfig.HIVE_LEG_LOW_X[2] < FieldConfig.HIVE_LEG_HIGH_X[0]);
    }

    @Test
    public void hiveTipping_blocksTheGapBetweenTheLegs() {
        PathPlanning tipping = PathPlanning.forField(BUFFER_MM, true);
        // the straight line down the gap was direct above; with the Hive tipping it must go around
        List<Point> path = tipping.plan(new Point(CENTER, 700), new Point(CENTER, 3000));
        assertTrue(path == null || path.size() > 1);
    }

    @Test
    public void hiveTipping_noPathEndsInsideTheHive() {
        PathPlanning tipping = PathPlanning.forField(BUFFER_MM, true);
        assertEquals(null, tipping.plan(new Point(300, 300), new Point(CENTER, CENTER)));
    }

    @Test
    public void hiveStill_gapIsOpenAgain() {
        PathPlanning still = PathPlanning.forField(BUFFER_MM, false);
        List<Point> path = still.plan(new Point(CENTER, 700), new Point(CENTER, 3000));
        assertNotNull(path);
        assertEquals(1, path.size());
    }

    @Test
    public void hiveTipping_routeAroundNeverEntersTheHiveFootprint() {
        PathPlanning tipping = PathPlanning.forField(BUFFER_MM, true);
        List<Point> path = tipping.plan(new Point(CENTER, 300), new Point(CENTER, 3350));
        assertNotNull(path);
        Point previous = new Point(CENTER, 300);
        for (Point next : path) {
            double length = Point.distance(previous, next);
            int steps = Math.max(1, (int) Math.ceil(length / 10));
            for (int i = 0; i <= steps; i++) {
                double t = (double) i / steps;
                double x = previous.getX() + (next.getX() - previous.getX()) * t;
                double y = previous.getY() + (next.getY() - previous.getY()) * t;
                boolean inside = x > FieldConfig.HIVE_MIN_X && x < FieldConfig.HIVE_MAX_X
                        && y > FieldConfig.HIVE_MIN_Y && y < FieldConfig.HIVE_MAX_Y;
                assertTrue("route enters the Hive at (" + x + ", " + y + ")", !inside);
            }
            previous = next;
        }
    }
}
