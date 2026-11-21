package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class TurretReadyTest {
    private static class FakeAlign extends TurretAutoAlign {
        boolean aligned;
        FakeAlign(boolean aligned) { super(null, null); this.aligned = aligned; }
        @Override public boolean isWithinRange() { return aligned; }
    }

    private static void step(TurretReady r) throws InterruptedException {
        r.updateCheckDone();
        Thread.sleep(5);
        r.updateCheckDone();
    }

    @Test
    public void aligned_done() throws InterruptedException {
        TurretReady r = new TurretReady(new FakeAlign(true));
        step(r);
        assertTrue(r.getIsDone());
        assertFalse(r.timedOut());
    }

    @Test
    public void timeoutWithHold_timedOutNotDone_thenDoneWhenAligned() throws InterruptedException {
        FakeAlign align = new FakeAlign(false);
        TurretReady r = new TurretReady(align, 1, true);
        step(r);
        assertTrue(r.timedOut());
        assertFalse(r.getIsDone());
        align.aligned = true;
        r.updateCheckDone();
        assertTrue(r.getIsDone());
    }

    @Test
    public void timeoutAuto_done() throws InterruptedException {
        TurretReady r = new TurretReady(new FakeAlign(false), 1, false);
        step(r);
        assertTrue(r.timedOut());
        assertTrue(r.getIsDone());
    }
}
