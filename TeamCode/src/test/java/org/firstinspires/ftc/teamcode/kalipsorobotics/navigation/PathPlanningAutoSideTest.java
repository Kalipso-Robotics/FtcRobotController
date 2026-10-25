package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.junit.Test;

import java.util.List;

/** AUTO (manual G402): an imaginary robot must never get to the opposing alliance's side. */
public class PathPlanningAutoSideTest {
    private static final double BUFFER_MM = 350;
    /** Imaginary robot: 18 in square, so its body is within 229 mm of its center. */
    private static final double ROBOT_HALF_MM = 229;
    /** The planner lets a segment run on a buffer edge, so allow a little over it. */
    private static final double TOLERANCE_MM = 2;
    private static final double LIMIT_X = FieldConfig.MIDLINE_X - ROBOT_HALF_MM;

    private final PathPlanning auto = PathPlanning.forFieldAuto(BUFFER_MM, ROBOT_HALF_MM);
    private final PathPlanning teleop = PathPlanning.forField(BUFFER_MM);

    @Test
    public void goalOnOpposingSide_hasNoPath() {
        assertNull(auto.plan(new Point(500, 3000), new Point(3000, 3000)));
    }

    @Test
    public void teleop_canCrossToTheOtherSide() {
        // same trip as above, with the AUTO block off
        assertNotNull(teleop.plan(new Point(500, 3000), new Point(3000, 3000)));
    }

    @Test
    public void auto_closesTheHiveGapBecauseItStraddlesTheMidline() {
        // the middle of the gap is on the midline, which is inside the blocked buffer
        assertNull(auto.plan(new Point(FieldConfig.MIDLINE_X, 700), new Point(FieldConfig.MIDLINE_X, 3000)));
    }

    @Test
    public void imaginaryRobot_neverEntersOpposingSide() {
        int plans = 0;
        for (double rx = 100; rx <= LIMIT_X; rx += 150) {
            for (double ry = 100; ry <= 3550; ry += 150) {
                for (double gx = 100; gx <= 3550; gx += 150) {
                    for (double gy = 100; gy <= 3550; gy += 150) {
                        Point robot = new Point(rx, ry);
                        Point goal = new Point(gx, gy);
                        List<Point> path = auto.plan(robot, goal);

                        if (gx > LIMIT_X + TOLERANCE_MM) {
                            assertNull("goal " + goal + " is on or too close to the opposing side", path);
                            continue;
                        }
                        if (path == null) {
                            continue;
                        }
                        plans++;
                        Point previous = robot;
                        for (Point next : path) {
                            assertStaysOnOwnSide(previous, next);
                            previous = next;
                        }
                    }
                }
            }
        }
        // make sure the loop actually checked plans and did not pass because everything was null
        assertTrue("only " + plans + " plans were checked", plans > 1000);
    }

    // ---- detours next to the Hive, with the AUTO block on ----

    @Test
    public void hiveEdge_launchPointToLaunchPoint_goesAroundTheOutside() {
        List<Point> path = auto.plan(FieldConfig.aLaunchPoint, FieldConfig.bLaunchPoint);
        assertNotNull(path);
        Point previous = FieldConfig.aLaunchPoint;
        for (Point next : path) {
            assertStaysOnOwnSide(previous, next);
            previous = next;
        }
        // the gap between the legs is closed, so the route must bend out past the low leg's buffer
        double legBufferLeftX = FieldConfig.HIVE_LEG_LOW_X[0] - BUFFER_MM;
        assertTrue(path.get(0).getX() <= legBufferLeftX);
    }

    @Test
    public void hiveEdge_launchPointsCanLeaveToTheGardenAndComeBack() {
        assertNotNull(auto.plan(FieldConfig.aLaunchPoint, FieldConfig.gardenPoint));
        assertNotNull(auto.plan(FieldConfig.gardenPoint, FieldConfig.aLaunchPoint));
        assertNotNull(auto.plan(FieldConfig.bLaunchPoint, 
                new Point(FieldConfig.bFlowerPoint.getX(), FieldConfig.bFlowerPoint.getY() + 500)));
    }

    @Test
    public void hiveEdge_goalJustOutsideTheLegBuffer_isReachable() {
        Point goal = new Point(FieldConfig.HIVE_LEG_LOW_X[0] - BUFFER_MM - 5, FieldConfig.MIDLINE_X);
        assertNotNull(auto.plan(FieldConfig.bLaunchPoint, goal));
    }

    @Test
    public void hiveEdge_goalJustInsideTheLegBuffer_isRefusedNotCrashed() {
        Point goal = new Point(FieldConfig.HIVE_LEG_LOW_X[0] - BUFFER_MM + 5, FieldConfig.MIDLINE_X);
        assertNull(auto.plan(FieldConfig.bLaunchPoint, goal));
    }

    @Test
    public void hiveEdge_everyPlanBetweenRealPlacesStaysOnOurSide() {
        Point[] places = {
                new Point(1389.5, 3486.5), FieldConfig.aLaunchPoint, FieldConfig.bLaunchPoint, FieldConfig.gardenPoint,
                new Point(FieldConfig.aFlowerPoint.getX() + 500, FieldConfig.aFlowerPoint.getY()),
                new Point(FieldConfig.bFlowerPoint.getX(), FieldConfig.bFlowerPoint.getY() + 500),
                new Point(100, 107.6)};
        for (Point from : places) {
            for (Point to : places) {
                List<Point> path = auto.plan(from, to);
                assertNotNull(from + " -> " + to, path);
                Point previous = from;
                for (Point next : path) {
                    assertStaysOnOwnSide(previous, next);
                    previous = next;
                }
            }
        }
    }

    /** Samples the segment every 10 mm so a path that clips across the midline is caught. */
    private static void assertStaysOnOwnSide(Point a, Point b) {
        double length = Point.distance(a, b);
        int steps = Math.max(1, (int) Math.ceil(length / 10));
        for (int i = 0; i <= steps; i++) {
            double t = (double) i / steps;
            double x = a.getX() + (b.getX() - a.getX()) * t;
            assertTrue("segment " + a + " -> " + b + " reaches x=" + x + ", past " + LIMIT_X,
                    x <= LIMIT_X + TOLERANCE_MM);
        }
    }
}
