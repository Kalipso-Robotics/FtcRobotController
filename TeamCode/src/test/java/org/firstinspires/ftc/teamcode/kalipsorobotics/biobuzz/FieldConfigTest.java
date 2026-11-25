package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.configs.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.junit.Test;

public class FieldConfigTest {
    private static final double FIELD = 3657.6;

    @Test
    public void nearestFlowerPicksTheClosest() {
        assertSame(FieldConfig.aFlowerPoint, FieldConfig.nearestFlower(new Point(300, 2400)));
        assertSame(FieldConfig.cFlowerPoint, FieldConfig.nearestFlower(new Point(3400, 1200)));
        assertSame(FieldConfig.bFlowerPoint, FieldConfig.nearestFlower(new Point(1200, 300)));
        assertSame(FieldConfig.dFlowerPoint, FieldConfig.nearestFlower(new Point(2400, 3300)));
    }

    @Test
    public void pilePointIsOffsetFromTheWallIntoTheField() {
        double d = FieldConfig.FLOWER_BALL_OFFSET_MM;
        assertPoint(FieldConfig.aFlowerPoint.getX() + d, FieldConfig.aFlowerPoint.getY(), FieldConfig.flowerPilePoint(FieldConfig.aFlowerPoint));
        assertPoint(FieldConfig.bFlowerPoint.getX(), FieldConfig.bFlowerPoint.getY() + d, FieldConfig.flowerPilePoint(FieldConfig.bFlowerPoint));
        assertPoint(FieldConfig.cFlowerPoint.getX() - d, FieldConfig.cFlowerPoint.getY(), FieldConfig.flowerPilePoint(FieldConfig.cFlowerPoint));
        assertPoint(FieldConfig.dFlowerPoint.getX(), FieldConfig.dFlowerPoint.getY() - d, FieldConfig.flowerPilePoint(FieldConfig.dFlowerPoint));
    }

    /** From anywhere outside the Hive footprint: opposite half, and at least a meter inside every wall. */
    @Test
    public void passTargetStaysFarInsideTheField() {
        for (double x = 100; x < FIELD; x += 100) {
            for (double y = 100; y < FIELD; y += 100) {
                if (y > FieldConfig.HIVE_MIN_Y && y < FieldConfig.HIVE_MAX_Y) continue;
                Point t = FieldConfig.passTarget(new Point(x, y));
                assertTrue((y > FIELD / 2) == (t.getY() < FIELD / 2));
                assertTrue(t.getX() >= FieldConfig.PASS_WALL_MARGIN_MM - 1e-9 && t.getX() <= FIELD - FieldConfig.PASS_WALL_MARGIN_MM + 1e-9);
                assertTrue(Math.min(t.getY(), FIELD - t.getY()) >= FieldConfig.PASS_Y_FROM_WALL_MM - 1e-9);
            }
        }
    }

    /** In line with the gap between the legs: straight across. Off to the side: the line still crosses inside the gap. */
    @Test
    public void passGoesBetweenTheHiveLegs() {
        Point straight = FieldConfig.passTarget(new Point(1800, 3000));
        assertEquals(1800, straight.getX(), 1e-9);

        Point robot = new Point(700, 3000);
        Point t = FieldConfig.passTarget(robot);
        double crossX = robot.getX() + (t.getX() - robot.getX()) * (FIELD / 2 - robot.getY()) / (t.getY() - robot.getY());
        assertTrue(crossX > FieldConfig.HIVE_LEG_LOW_X[2] && crossX < FieldConfig.HIVE_LEG_HIGH_X[0]);
    }

    private static void assertPoint(double x, double y, Point p) {
        assertEquals(x, p.getX(), 1e-9);
        assertEquals(y, p.getY(), 1e-9);
    }
}
