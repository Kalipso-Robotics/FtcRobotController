package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter.ShooterDataCollector.Oscillation;
import org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter.ShooterDataCollector.ResponseTracker;
import org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter.ShooterDataCollector.SteadyWindow;
import org.junit.Test;

import java.util.function.DoubleUnaryOperator;

public class ShooterDataCollectorTest {
    private static final double CYCLE = 2 * Math.PI / 500; // 500 ms period

    private static SteadyWindow filled(double durationMs, DoubleUnaryOperator rpsAtMs) {
        SteadyWindow window = new SteadyWindow(3000);
        for (double ms = 0; ms <= durationMs; ms += 20) {
            window.add(ms, rpsAtMs.applyAsDouble(ms), 12.5);
        }
        return window;
    }

    @Test
    public void steady_waitsForTheFullWindowAndIgnoresEncoderSteps() {
        DoubleUnaryOperator noise = ms -> 40 + (ms / 20 % 2) * 0.71;
        assertFalse(filled(2980, noise).isSteady(0.5));
        SteadyWindow window = filled(3000, noise);
        assertTrue(window.isSteady(0.5));
        assertEquals(40.35, window.rpsAvg(), 0.02);
        assertEquals(0.71, window.spread(), 1e-9);
        assertFalse(window.oscillation().sustained);
        window.clear();
        assertFalse(window.isSteady(0.5));
        assertEquals(0, window.rpsAvg(), 0);
        window.add(9000, 40, 12.5);
        assertFalse(window.isSteady(0.5)); // clear restarts the wait
    }

    @Test
    public void steady_rejectsASlowClimb() {
        assertFalse(filled(6000, ms -> 40 + ms * 0.0005).isSteady(0.5)); // 0.5 RPS per second
    }

    @Test
    public void oscillation_sustainedIsDetectedWithItsPeriodAndAmplitude() {
        Oscillation o = filled(4000, ms -> 40 + 3 * Math.sin(ms * CYCLE)).oscillation();
        assertTrue(o.sustained);
        assertEquals(500, o.periodMs, 25);
        assertEquals(7.5, o.amplitudePct, 0.2);
    }

    @Test
    public void oscillation_dyingOutOrSmallOrASingleDipIsNotSustained() {
        assertFalse(filled(3000, ms -> 40 + 8 * Math.exp(-ms / 1500) * Math.sin(ms * CYCLE)).oscillation().sustained);
        assertFalse(filled(4000, ms -> 40 + 0.7 * Math.sin(ms * CYCLE)).oscillation().sustained);
        assertFalse(filled(4000, ms -> ms > 2800 && ms < 3200 ? 34 : 40).oscillation().sustained);
        assertFalse(filled(1500, ms -> 40 + 3 * Math.sin(ms * CYCLE)).oscillation().sustained); // window not full
        assertFalse(new SteadyWindow(3000).oscillation().sustained);
    }

    @Test
    public void tracker_timesAStepThenADip() {
        ResponseTracker tracker = new ResponseTracker();
        assertEquals(ResponseTracker.Event.stepStart, tracker.update(0, 40, 0));
        assertEquals(ResponseTracker.Event.NONE, tracker.update(500, 40, 30));
        assertEquals(ResponseTracker.Event.NONE, tracker.update(800, 40, 42));   // reached, overshot by 2
        assertEquals(ResponseTracker.Event.NONE, tracker.update(1000, 40, 40.5)); // inside from here
        assertEquals(ResponseTracker.Event.stepDone, tracker.update(1300, 40, 40));
        assertEquals("rise=800ms settle=1000ms overshootPct=5.0", tracker.lastStep);

        assertEquals(ResponseTracker.Event.dipStart, tracker.update(2000, 40, 37));
        assertEquals(ResponseTracker.Event.NONE, tracker.update(2050, 40, 34));
        assertEquals(ResponseTracker.Event.NONE, tracker.update(2400, 40, 39.5));
        assertEquals(ResponseTracker.Event.dipDone, tracker.update(2700, 40, 40));
        assertEquals("dropPct=15.0 recover=400ms overshootPct=0.0", tracker.lastDip);
    }

    @Test
    public void tracker_oscillatingNeverSettlesAndSpinDownOvershootCountsDownward() {
        ResponseTracker tracker = new ResponseTracker();
        tracker.update(0, 40, 60);
        for (int ms = 100; ms < 5000; ms += 100) {
            assertEquals(ResponseTracker.Event.NONE, tracker.update(ms, 40, ms / 100 % 2 == 0 ? 37 : 43));
        }
        assertTrue(tracker.summary(), tracker.summary().contains("settling(4900ms) overshootPct=7.5"));
        assertEquals("none yet", tracker.lastStep);
    }
}
