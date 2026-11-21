package org.firstinspires.ftc.teamcode.kalipsorobotics.actions;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShootAllAction;
import org.junit.Test;

public class ShootAbortTest {
    // args: abortOnTimeout, shooterTimedOut, turretTimedOut, shooterDone, turretDone

    @Test
    public void abortOff_neverAborts() {
        assertFalse(ShootAllAction.shouldAbortOnTimeout(false, true, true, false, false));
    }

    @Test
    public void gateTimedOutAndNotReady_aborts() {
        assertTrue(ShootAllAction.shouldAbortOnTimeout(true, false, true, true, false));
        assertTrue(ShootAllAction.shouldAbortOnTimeout(true, true, false, false, true));
    }

    @Test
    public void bothGatesReadyOnTheTimeoutLoop_feeds() {
        // ready is ready: a timeout flag set the same loop must not cut off a legit feed mid-sequence
        assertFalse(ShootAllAction.shouldAbortOnTimeout(true, true, false, true, true));
    }

    @Test
    public void noTimeout_noAbort() {
        assertFalse(ShootAllAction.shouldAbortOnTimeout(true, false, false, false, false));
    }
}
