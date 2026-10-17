package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.junit.Test;

import java.util.List;

public class PathPlanningTest {
    // Hive 1224..1824 x 1529..2129, buffer 300 -> buffered box 924..2124 x 1229..2429
    private final PathPlanning planner = new PathPlanning(1224, 1529, 1824, 2129, 300);

    @Test
    public void sameSide_isDirect() {
        List<Point> path = planner.plan(new Point(200, 200), new Point(200, 1000));
        assertNotNull(path);
        assertEquals(1, path.size());
    }

    @Test
    public void adjacentSides_usesOneCorner() {
        // robot left of the hive, goal below it
        List<Point> path = planner.plan(new Point(500, 1800), new Point(1500, 600));
        assertNotNull(path);
        assertEquals(2, path.size());
        // bottom-left buffered corner
        assertTrue(path.get(0).getX() < 924);
        assertTrue(path.get(0).getY() < 1229);
    }

    @Test
    public void oppositeSides_usesTwoCornersOnShorterSide_bottom() {
        List<Point> path = planner.plan(new Point(500, 1800), new Point(3000, 1800));
        assertNotNull(path);
        assertEquals(3, path.size());
        assertTrue(path.get(0).getY() < 1229);
    }

    @Test
    public void oppositeSides_usesTwoCornersOnShorterSide_top() {
        List<Point> path = planner.plan(new Point(500, 2000), new Point(3000, 2000));
        assertNotNull(path);
        assertEquals(3, path.size());
        assertTrue(path.get(0).getY() > 2429);
    }

    @Test
    public void cornerOutsideField_forcesOtherSide() {
        // buffered box reaches y=-200, so the bottom corners are off the field
        PathPlanning nearWall = new PathPlanning(1224, 100, 1824, 700, 300);
        List<Point> path = nearWall.plan(new Point(500, 400), new Point(3000, 400));
        assertNotNull(path);
        assertEquals(3, path.size());
        assertTrue(path.get(0).getY() > 1000);
    }

    @Test
    public void goalInsideBuffer_returnsNull() {
        assertNull(planner.plan(new Point(500, 1800), new Point(1500, 1800)));
    }

    @Test
    public void robotInsideBuffer_returnsNull() {
        assertNull(planner.plan(new Point(1500, 1800), new Point(200, 200)));
    }

    @Test
    public void everySegmentIsClear() {
        for (int rx = 100; rx <= 3500; rx += 400) {
            for (int ry = 100; ry <= 3500; ry += 400) {
                for (int gx = 100; gx <= 3500; gx += 400) {
                    for (int gy = 100; gy <= 3500; gy += 400) {
                        Point robot = new Point(rx, ry);
                        Point goal = new Point(gx, gy);
                        List<Point> path = planner.plan(robot, goal);
                        if (path == null) {
                            continue;
                        }
                        Point previous = robot;
                        boolean first = true;
                        for (Point next : path) {
                            // a robot that starts inside a ring first drives straight out of it
                            if (!(first && planner.insideBuffer(robot))) {
                                assertTrue("blocked " + previous + " -> " + next, planner.clear(previous, next));
                            }
                            first = false;
                            previous = next;
                        }
                        assertEquals(goal, path.get(path.size() - 1));
                    }
                }
            }
        }
    }

    @Test
    public void secondBox_isAvoidedToo() {
        // extra box 2400..2600 x 1700..1900 -> buffered 2100..2900 x 1400..2200
        PathPlanning two = new PathPlanning(300, new double[]{1224, 1529, 1824, 2129},
                new double[]{2400, 1700, 2600, 1900});
        Point robot = new Point(500, 1800);
        Point goal = new Point(3300, 1800);
        List<Point> path = two.plan(robot, goal);
        assertNotNull(path);
        Point previous = robot;
        for (Point next : path) {
            assertTrue("blocked " + previous + " -> " + next, two.clear(previous, next));
            previous = next;
        }
        assertEquals(goal, path.get(path.size() - 1));
    }

    @Test
    public void goalInsideSecondBoxBuffer_returnsNull() {
        PathPlanning two = new PathPlanning(300, new double[]{1224, 1529, 1824, 2129},
                new double[]{2400, 1700, 2600, 1900});
        assertNull(two.plan(new Point(500, 1800), new Point(2500, 1800)));
    }

    @Test
    public void realField_flowerAIsAvoided() {
        PathPlanning field = PathPlanning.forField(350);
        Point robot = new Point(0, 1800);
        Point goal = new Point(3100, 3000);
        List<Point> path = field.plan(robot, goal);
        assertNotNull(path);
        Point previous = robot;
        for (Point next : path) {
            assertTrue("blocked " + previous + " -> " + next, field.clear(previous, next));
            previous = next;
        }
        System.out.println("flower test path " + path);
    }
}
