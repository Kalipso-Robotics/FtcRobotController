package org.firstinspires.ftc.teamcode.kalipsorobotics.localization;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class ResetOdometryToPosTest {
    private static final Position CORNER = new Position(278, 188, 0);
    private static final Position STALE = new Position(244, 6427, Math.toRadians(-154.9)); // 2026-11-26 TeleOp log

    @Test
    public void rewritesWhenOdometryThreadClobbersTheReset() {
        SharedData.setOdometryWheelIMUPosition(STALE);
        ResetOdometryToPos reset = new ResetOdometryToPos(CORNER);

        reset.updateCheckDone();                              // try 1 writes the corner
        assertEquals(278, SharedData.getOdometryWheelIMUPosition().getX(), 1e-9);
        SharedData.setOdometryWheelIMUPosition(STALE);        // thread read before the reset, wrote after it
        reset.updateCheckDone();                              // sees the stale pose, writes again
        assertFalse(reset.getIsDone());
        reset.updateCheckDone();                              // landed now
        assertTrue(reset.getIsDone());
        assertEquals(188, SharedData.getOdometryWheelIMUPosition().getY(), 1e-9);
    }

    @Test
    public void givesUpInsteadOfDraggingAMovingRobotBack() {
        ResetOdometryToPos reset = new ResetOdometryToPos(CORNER);
        for (int i = 0; i < 10 && !reset.getIsDone(); i++) {
            reset.updateCheckDone();
            SharedData.setOdometryWheelIMUPosition(STALE);    // never lands
        }
        assertTrue(reset.getIsDone());
    }
}
