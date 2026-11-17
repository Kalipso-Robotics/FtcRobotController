package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.junit.Test;

/** planPickup with a forced intake side: Flower piles must use the side the flower is on. */
public class PathPlanningForceSideTest {
    private final PathPlanning auto = PathPlanning.forFieldAuto(350, 229);
    private final Point robot = new Point(500, 1800);
    private final Point ball = new Point(1000, 1800);

    @Test
    public void forcedFront_neverUsesBack() {
        // Facing away from the ball, the back would be cheaper; forcing the front must still pick the front.
        assertTrue("precondition: free choice picks the back here", auto.planPickup(robot, Math.PI, ball).useBackIntake);
        PickupPlan plan = auto.planPickup(robot, Math.PI, ball, false);
        assertTrue(plan.why, plan.isOk());
        assertFalse(plan.useBackIntake);
    }

    @Test
    public void forcedBack_alwaysUsesBack() {
        PickupPlan plan = auto.planPickup(robot, 0, ball, true);
        assertTrue(plan.why, plan.isOk());
        assertTrue(plan.useBackIntake);
    }

    @Test
    public void nullForce_sameAsBestOfBoth() {
        PickupPlan free = auto.planPickup(robot, Math.PI, ball);
        PickupPlan nullForce = auto.planPickup(robot, Math.PI, ball, null);
        assertEquals(free.useBackIntake, nullForce.useBackIntake);
        assertEquals(free.standOff.getX(), nullForce.standOff.getX(), 1e-9);
        assertEquals(free.standOff.getY(), nullForce.standOff.getY(), 1e-9);
    }
}
