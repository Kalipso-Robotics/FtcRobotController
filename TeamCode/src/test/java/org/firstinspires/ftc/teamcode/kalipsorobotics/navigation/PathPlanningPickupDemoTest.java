package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.junit.Test;

/**
 * Prints what planPickup does for balls in deadzones, in buffer rings and elsewhere, so the results
 * can be read. Cases with a clear expected answer are also asserted.
 */
public class PathPlanningPickupDemoTest {
    private static final double BUFFER_MM = 350;
    private static final double ROBOT_HALF_MM = 229;

    private final PathPlanning teleop = PathPlanning.forField(BUFFER_MM);
    private final PathPlanning auto = PathPlanning.forFieldAuto(BUFFER_MM, ROBOT_HALF_MM);

    private PickupPlan show(String label, PathPlanning planner, Point robot, Point ball, PickupPlan.Reason expected) {
        PickupPlan plan = planner.planPickup(robot, ball);
        System.out.printf("%-48s ball (%4.0f,%4.0f)  %s%n", label, ball.getX(), ball.getY(), plan);
        if (plan.isOk()) {
            System.out.println("      waypoints " + plan.waypoints + " facing "
                    + Math.round(Math.toDegrees(plan.finalHeadingRad)) + " deg");
        }
        if (expected != null) {
            assertEquals(label, expected, plan.reason);
        }
        return plan;
    }

    @Test
    public void printCases() {
        Point robot = new Point(500, 1800);
        System.out.println("robot starts at " + robot + ", buffer " + BUFFER_MM + ", grab point "
                + RobotFootprint.FRONT_GRAB_MM + " mm ahead of the robot center");

        System.out.println("--- deadzones (inside corners of the legs) ---");
        show("deadzone: low leg, low-Y corner", teleop, robot, new Point(1400, 1380), PickupPlan.Reason.IN_DEADZONE);
        show("deadzone: low leg, high-Y corner", teleop, robot, new Point(1400, 2280), PickupPlan.Reason.IN_DEADZONE);
        show("deadzone: high leg, low-Y corner", teleop, robot, new Point(2250, 1380), PickupPlan.Reason.IN_DEADZONE);
        show("deadzone: high leg, high-Y corner", teleop, robot, new Point(2250, 2280), PickupPlan.Reason.IN_DEADZONE);
        show("just outside the deadzone (110 mm from corner)", teleop, robot, new Point(1340.6 + 110, 1380), null);

        System.out.println("--- in a buffer ring ---");
        show("ring, 100 mm from low leg (stands off outside)", teleop, robot, new Point(1100, 1800), PickupPlan.Reason.OK);
        show("ring, 70 mm from low leg (drives into ring)", teleop, robot, new Point(1130, 1800), PickupPlan.Reason.OK);
        show("ring, 50 mm from low leg (front touches, pulls in)", teleop, robot, new Point(1150, 1800), PickupPlan.Reason.OK);
        show("ball touching the low leg (pulls in)", teleop, robot, new Point(1165.1, 1800), PickupPlan.Reason.OK);
        show("ring, outer side of high leg, 100 mm", teleop, new Point(3000, 1800), new Point(2557, 1800), null);
        show("ring, near flower A (155 mm from its face)", teleop, new Point(900, 2900), new Point(480, 2500), null);
        show("ring, in the gap near a leg (160 mm from face)", teleop, new Point(1829, 700), new Point(1500, 1380), null);

        System.out.println("--- the open middle of the gap ---");
        show("middle of the gap, from below", teleop, new Point(1829, 700), new Point(1829, 1800), PickupPlan.Reason.OK);

        System.out.println("--- inside an obstacle ---");
        show("inside the low leg", teleop, robot, new Point(1270, 1800), PickupPlan.Reason.INSIDE_OBSTACLE);

        System.out.println("--- AUTO (stay on own side) ---");
        show("AUTO: ball on the opposing side", auto, robot, new Point(2560, 1800), PickupPlan.Reason.OPPOSING_SIDE);
        show("AUTO: ring ball on own side", auto, robot, new Point(1130, 1800), PickupPlan.Reason.OK);
        show("AUTO: 29 mm from the midline (pulls in)", auto, new Point(500, 3000), new Point(1800, 3000), PickupPlan.Reason.OK);
        show("ball in a field corner", teleop, robot, new Point(30, 30), PickupPlan.Reason.NO_APPROACH);
        show("AUTO: 300 mm from the midline", auto, new Point(500, 3000), new Point(1529, 3000), PickupPlan.Reason.OK);

        System.out.println("--- ten pictured cases ---");
        double mid = 3657.6 / 2;
        double r = RobotFootprint.BALL_RADIUS_MM;
        double lowOuter = FieldConfig.HIVE_LEG_LOW_X[0];
        double lowInner = FieldConfig.HIVE_LEG_LOW_X[2];
        double highInner = FieldConfig.HIVE_LEG_HIGH_X[0];
        double highOuter = FieldConfig.HIVE_LEG_HIGH_X[2];
        show("P1 two corners, around the bottom", teleop, new Point(500, mid), new Point(3100, mid), PickupPlan.Reason.OK);
        show("P2 two corners, around the top", teleop, new Point(800, 2150), new Point(3100, 2150), PickupPlan.Reason.OK);
        show("P3 ball against the low leg, outer face", teleop, new Point(500, mid), new Point(lowOuter - r, mid), PickupPlan.Reason.OK);
        show("P4 ball against the high leg, outer face", teleop, new Point(3300, mid), new Point(highOuter + r, mid), PickupPlan.Reason.OK);
        show("P5 ball against the low leg, inner face", teleop, new Point(mid, 600), new Point(lowInner + r, mid), PickupPlan.Reason.OK);
        show("P6 ball against the high leg, inner face", teleop, new Point(mid, 3000), new Point(highInner - r, mid), PickupPlan.Reason.OK);
        show("P7 ball in a deadzone", teleop, new Point(500, mid), new Point(lowInner + 50, FieldConfig.HIVE_MIN_Y + 50), PickupPlan.Reason.IN_DEADZONE);
        show("P8 ball just outside the deadzone", teleop, new Point(500, mid), new Point(lowInner + 110, FieldConfig.HIVE_MIN_Y + 46), null);
        show("P9 straight through the gap", teleop, new Point(mid, 600), new Point(mid, 3000), PickupPlan.Reason.OK);
        show("P10 AUTO ball against the low leg", auto, new Point(800, 1500), new Point(lowOuter - r, 2200), PickupPlan.Reason.OK);
    }
}
