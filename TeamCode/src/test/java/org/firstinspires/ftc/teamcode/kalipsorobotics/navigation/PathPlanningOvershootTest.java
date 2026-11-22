package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.junit.Test;

/** overshoot pushes the stop toward the ball, and never into a wall or a Hive leg. */
public class PathPlanningOvershootTest {
    private final PathPlanning auto = PathPlanning.forFieldAuto(350, 229);
    private final Point robot = new Point(300, 1000);
    private final Point openBall = new Point(800, 1000); // open floor, away from the legs and flowers

    @Test
    public void frontIntake_movesTheStopTowardTheBall() {
        PickupPlan plan = auto.planPickup(robot, 0, openBall, false);
        assertTrue(plan.why, plan.isOk());
        Point past = auto.overshoot(plan, 80);
        assertEquals(80, Point.distance(plan.standOff, past), 1e-6);
        assertTrue("closer to the ball", Point.distance(past, openBall) < Point.distance(plan.standOff, openBall));
    }

    @Test
    public void backIntake_alsoMovesTowardTheBall() {
        PickupPlan plan = auto.planPickup(robot, 0, openBall, true);
        assertTrue(plan.why, plan.isOk());
        Point past = auto.overshoot(plan, 80);
        assertEquals(80, Point.distance(plan.standOff, past), 1e-6);
        assertTrue("closer to the ball", Point.distance(past, openBall) < Point.distance(plan.standOff, openBall));
    }

    @Test
    public void zeroOvershoot_isTheStandOff() {
        PickupPlan plan = auto.planPickup(robot, 0, openBall);
        assertEquals(plan.standOff, auto.overshoot(plan, 0));
    }

    @Test
    public void againstTheWall_staysAtTheStandOff() {
        PickupPlan plan = auto.planPickup(robot, 0, new Point(1000, 3550));
        assertTrue(plan.why, plan.isOk());
        assertEquals(plan.standOff, auto.overshoot(plan, 2000));
    }

    @Test
    public void nextToAHiveLeg_staysAtTheStandOff() {
        // The planner backs in diagonally here to clear a leg; 80 mm more would put the body into it.
        PickupPlan plan = auto.planPickup(new Point(500, 1800), 0, new Point(1000, 1800), true);
        assertTrue(plan.why, plan.isOk());
        assertEquals(plan.standOff, auto.overshoot(plan, 80));
    }
}
