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
}
