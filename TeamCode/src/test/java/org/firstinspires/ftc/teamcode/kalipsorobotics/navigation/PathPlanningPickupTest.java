package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/** Ball pickup: balls in the buffer ring are reachable with a stand-off, except in deadzones. */
public class PathPlanningPickupTest {
    private static final double BUFFER_MM = 350;
    private static final double ROBOT_HALF_MM = 229;

    private final PathPlanning teleop = PathPlanning.forField(BUFFER_MM);
    private final PathPlanning auto = PathPlanning.forFieldAuto(BUFFER_MM, ROBOT_HALF_MM);

    // Low-X leg: x 1200.6..1340.6, y 1334.1..2323.5. Its buffer ring reaches x = 850.6.

    @Test
    public void ballJustOutsideTheRing_standsOffOutsideTheRing() {
        // 100 mm from the leg face: the stand-off (260 mm short of the ball) is at x = 840, outside the ring
        PickupPlan plan = teleop.planPickup(new Point(500, 1800), new Point(1100, 1800));
        assertTrue(plan.toString(), plan.isOk());
        assertFalse(plan.enteredRing);
        assertEquals(840, plan.standOff.getX(), 1e-6);
        assertEquals(1800, plan.standOff.getY(), 1e-6);
        assertEquals(0, plan.finalHeadingRad, 1e-9);
    }

    @Test
    public void ballInTheRing_stillReachableByDrivingIn() {
        // 70 mm from the leg face: the stand-off at x = 870 is inside the ring
        PickupPlan plan = teleop.planPickup(new Point(500, 1800), new Point(1130, 1800));
        assertTrue(plan.toString(), plan.isOk());
        assertTrue(plan.enteredRing);
        assertEquals(870, plan.standOff.getX(), 1e-6);
        assertEquals(0, plan.finalHeadingRad, 1e-9);
        // the intake grabs the ball: stand-off + GRAB_MM ahead = the ball
        assertEquals(1130, plan.standOff.getX() + RobotFootprint.GRAB_MM * Math.cos(plan.finalHeadingRad), 1e-6);
    }

    @Test
    public void ballCloseToTheLeg_isPickedUpByPullingIn() {
        // 50 mm from the leg face: the ball cannot go inside the mouth without the tip hitting the leg,
        // but the front can touch the ball and pull it in
        PickupPlan plan = teleop.planPickup(new Point(500, 1800), new Point(1150, 1800));
        assertTrue(plan.toString(), plan.isOk());
        assertTrue(plan.pullIn);
        assertEquals(1150 - RobotFootprint.pullInMm(), plan.standOff.getX(), 1e-6);
        assertEquals(0, plan.finalHeadingRad, 1e-9);
    }

    @Test
    public void ballTouchingTheLeg_isPickedUpByPullingIn() {
        // the ball center is one ball radius from the leg face (x = 1200.6)
        double ballX = FieldConfig.HIVE_LEG_LOW_X[0] - RobotFootprint.BALL_RADIUS_MM;
        PickupPlan plan = teleop.planPickup(new Point(500, 1800), new Point(ballX, 1800));
        assertTrue(plan.toString(), plan.isOk());
        assertTrue(plan.pullIn);
        // the front tip ends up a whole ball (71 mm) from the leg, clear of the 20 mm margin
        double tipX = plan.standOff.getX() + RobotFootprint.FRONT_MM;
        assertEquals(FieldConfig.HIVE_LEG_LOW_X[0] - 2 * RobotFootprint.BALL_RADIUS_MM, tipX, 1e-6);
    }

    @Test
    public void ballWithRoomForTheMouth_doesNotUsePullIn() {
        PickupPlan plan = teleop.planPickup(new Point(500, 1800), new Point(1100, 1800));
        assertTrue(plan.isOk());
        assertFalse(plan.pullIn);
    }

    @Test
    public void ballInAFieldCorner_hasNoApproach() {
        // the body is wider than the gap to both walls at every angle
        PickupPlan plan = teleop.planPickup(new Point(500, 1800), new Point(30, 30));
        assertFalse(plan.isOk());
        assertEquals(PickupPlan.Reason.NO_APPROACH, plan.reason);
    }

    @Test
    public void ballInADeadzone_isRejected() {
        // inside corner of the low-X leg at the low-Y end
        PickupPlan plan = teleop.planPickup(new Point(500, 1800), new Point(1400, 1380));
        assertEquals(PickupPlan.Reason.IN_DEADZONE, plan.reason);
    }

    @Test
    public void ballInsideALeg_isRejected() {
        PickupPlan plan = teleop.planPickup(new Point(500, 1800), new Point(1270, 1800));
        assertEquals(PickupPlan.Reason.INSIDE_OBSTACLE, plan.reason);
    }

    @Test
    public void ballOffTheField_isRejected() {
        assertEquals(PickupPlan.Reason.OFF_FIELD,
                teleop.planPickup(new Point(500, 1800), new Point(-10, 1800)).reason);
    }

    @Test
    public void auto_ballOnTheOpposingSide_isRejected() {
        PickupPlan plan = auto.planPickup(new Point(500, 500), new Point(2500, 500));
        assertEquals(PickupPlan.Reason.OPPOSING_SIDE, plan.reason);
    }

    @Test
    public void teleop_ballOnTheOtherSide_isReachable() {
        assertTrue(teleop.planPickup(new Point(500, 3000), new Point(2500, 3000)).isOk());
    }

    @Test
    public void auto_ballOnOwnSide_bodyStaysOnOwnSide() {
        PickupPlan plan = auto.planPickup(new Point(500, 3000), new Point(1700, 3000));
        assertTrue(plan.toString(), plan.isOk());
        for (double[] corner : RobotFootprint.corners(plan.standOff, plan.finalHeadingRad)) {
            assertTrue(corner[0] <= FieldConfig.MIDLINE_X + 1);
        }
    }

    @Test
    public void auto_ballNextToTheMidline_isPulledInWithTheBodyOnOwnSide() {
        // 29 mm from the midline: the mouth cannot take it without the body crossing the line, but the
        // front can touch it and pull it in, with the whole body still on its own side
        PickupPlan plan = auto.planPickup(new Point(500, 3000), new Point(1800, 3000));
        assertTrue(plan.toString(), plan.isOk());
        assertTrue(plan.pullIn);
        for (double[] corner : RobotFootprint.corners(plan.standOff, plan.finalHeadingRad)) {
            assertTrue(corner[0] <= FieldConfig.MIDLINE_X + 1);
        }
    }

    @Test
    public void robotInsideABuffer_hasNoPath() {
        PickupPlan plan = teleop.planPickup(new Point(1000, 1800), new Point(500, 500));
        assertEquals(PickupPlan.Reason.NO_PATH, plan.reason);
    }

    @Test
    public void randomBalls_neverTouchAnObstacle() {
        for (PathPlanning planner : new PathPlanning[]{teleop, auto}) {
            boolean isAuto = planner == auto;
            Random random = new Random(7);
            int checked = 0;
            for (int trial = 0; trial < 4000; trial++) {
                Point robot = new Point(100 + random.nextDouble() * 1400, 100 + random.nextDouble() * 3400);
                Point ball = new Point(50 + random.nextDouble() * 3550, 50 + random.nextDouble() * 3550);
                PickupPlan plan = planner.planPickup(robot, ball);
                if (!plan.isOk()) {
                    continue;
                }
                checked++;

                // the intake grabs the ball
                double reach = plan.pullIn ? RobotFootprint.pullInMm() : RobotFootprint.GRAB_MM;
                double grabX = plan.standOff.getX() + reach * Math.cos(plan.finalHeadingRad);
                double grabY = plan.standOff.getY() + reach * Math.sin(plan.finalHeadingRad);
                assertEquals(ball.getX(), grabX, 1e-6);
                assertEquals(ball.getY(), grabY, 1e-6);

                // everything up to the last stretch stays out of the buffers
                List<Point> path = plan.waypoints;
                assertEquals(plan.standOff, path.get(path.size() - 1));
                Point previous = robot;
                for (int i = 0; i < path.size(); i++) {
                    boolean lastStretch = plan.enteredRing && i == path.size() - 1;
                    if (!lastStretch) {
                        assertTrue("blocked " + previous + " -> " + path.get(i), planner.clear(previous, path.get(i)));
                    }
                    previous = path.get(i);
                }

                // the body, sampled over the whole last stretch, never touches a real obstacle
                Point from = plan.enteredRing ? path.get(path.size() - 2) : plan.standOff;
                double length = Point.distance(from, plan.standOff);
                int steps = Math.max(1, (int) Math.ceil(length / 10));
                for (int step = 0; step <= steps; step++) {
                    double t = (double) step / steps;
                    Point pose = new Point(from.getX() + (plan.standOff.getX() - from.getX()) * t,
                            from.getY() + (plan.standOff.getY() - from.getY()) * t);
                    assertBodyIsClear(pose, plan.finalHeadingRad, isAuto);
                }
            }
            assertTrue("only " + checked + " plans were checked", checked > 200);
        }
    }

    /** Samples points all over the robot body and checks none is inside a real obstacle. */
    private static void assertBodyIsClear(Point pose, double heading, boolean isAuto) {
        List<double[]> obstacles = new ArrayList<>();
        double half = FieldConfig.FLOWER_SIZE_MM / 2;
        for (Point flower : new Point[]{FieldConfig.aFlowerPoint, FieldConfig.bFlowerPoint,
                FieldConfig.cFlowerPoint, FieldConfig.dFlowerPoint}) {
            obstacles.add(new double[]{flower.getX() - half, flower.getY() - half,
                    flower.getX() + half, flower.getY() + half});
        }
        obstacles.add(FieldConfig.HIVE_LEG_LOW_X);
        obstacles.add(FieldConfig.HIVE_LEG_HIGH_X);
        for (double[] zone : FieldConfig.HIVE_DEADZONES) {
            obstacles.add(zone);
        }

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double halfWidth = RobotFootprint.WIDTH_MM / 2;
        for (double u = -RobotFootprint.BACK_MM; u <= RobotFootprint.FRONT_MM; u += 15) {
            for (double v = -halfWidth; v <= halfWidth; v += 15) {
                double x = pose.getX() + u * cos - v * sin;
                double y = pose.getY() + u * sin + v * cos;
                assertTrue("body is off the field at " + x + "," + y, x >= 0 && x <= 3657.6 && y >= 0 && y <= 3657.6);
                if (isAuto) {
                    assertTrue("body crosses the midline at x=" + x, x <= FieldConfig.MIDLINE_X + 1);
                }
                for (double[] box : obstacles) {
                    assertFalse("body at " + x + "," + y + " is inside an obstacle",
                            x > box[0] && x < box[2] && y > box[1] && y < box[3]);
                }
            }
        }
    }
}
