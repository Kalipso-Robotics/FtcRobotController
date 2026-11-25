package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

/** KpTuner against a simulated flywheel: first order lag, dead time, power 0..1, quantized encoder. */
public class KpTunerTest {
    private static final double KF = 0.0113;
    private static final double[] TARGETS = {21, 26, 31, 36, 41, 46, 51, 56};

    /** kfError: the motor needs this many times KF to hold a speed. */
    private static KpTuner.Trial simulate(double target, double kp, double tauS, double delayS, double kfError) {
        double dt = 0.02, rps = target - 10;
        int delay = (int) Math.round(delayS / dt);
        List<Double> history = new ArrayList<>(), t = new ArrayList<>(), r = new ArrayList<>();
        for (int i = 0; i * dt < 4; i++) {
            history.add(Math.round(rps / 0.714) * 0.714);
            double seen = history.get(Math.max(0, i - delay));
            t.add(i * dt * 1000);
            r.add(seen);
            double power = Math.max(0, Math.min(1, KF * target + kp * (target - seen)));
            rps += dt / tauS * (power / (KF * kfError) - rps);
        }
        return KpTuner.analyze(target, kp, t, r);
    }

    @Test
    public void findsAFastCalmKp() {
        KpTuner tuner = new KpTuner("test", KF, TARGETS, (target, kp) -> simulate(target, kp, 0.8, 0.2, 1));
        assertTrue(tuner.run());
        assertTrue(tuner.isVerified());
        assertEquals(TARGETS.length, tuner.getTargets().size());
        for (KpTuner.Target target : tuner.getTargets()) {
            assertTrue("the search must find where it starts to oscillate", target.foundOscillation);
            assertTrue(target.best.kp <= target.limitKp);
        }
        for (double target : TARGETS) {
            KpTuner.Trial best = simulate(target, tuner.getBestKp(), 0.8, 0.2, 1);
            KpTuner.Trial low = simulate(target, 0.2 * KF, 0.8, 0.2, 1);
            assertFalse(best.oscillating);
            assertTrue("faster than the starting kP: " + best.settleMs + " vs " + low.settleMs, best.settleMs < low.settleMs);
        }
    }

    @Test
    public void kfOff_needsMoreKpToReachTheTarget_stillFoundAndCalm() {
        KpTuner tuner = new KpTuner("test", KF, TARGETS, (target, kp) -> simulate(target, kp, 0.8, 0.2, 1.05));
        assertTrue(tuner.run());
        assertTrue(tuner.isVerified());
        assertTrue(Double.isNaN(simulate(56, 0.2 * KF, 0.8, 0.2, 1.05).settleMs));
        assertTrue(tuner.getBestKp() > 0.2 * KF);
    }

    @Test
    public void startingKpOscillates_searchesDown() {
        double limit = 0.1 * KF;
        KpTuner tuner = new KpTuner("test", KF, new double[]{30}, (target, kp) ->
                new KpTuner.Trial(target, kp, 500, 0, 0, kp > limit));
        assertTrue(tuner.run());
        assertTrue(tuner.getBestKp() <= limit);
        assertTrue(tuner.isVerified());
    }

    @Test
    public void averageIsCappedAtTheLowestLimit() {
        // 50 RPS oscillates far earlier than 20 RPS: the average of the bests would oscillate at 50.
        KpTuner tuner = new KpTuner("test", KF, new double[]{20, 50}, (target, kp) ->
                new KpTuner.Trial(target, kp, 1 / kp, 0, 0, kp > (target > 30 ? 0.5 : 4) * KF));
        assertTrue(tuner.run());
        assertTrue(tuner.getAverageKp() > 0.5 * KF);
        assertTrue(tuner.getBestKp() <= 0.5 * KF);
        assertTrue(tuner.isVerified());
    }

    @Test
    public void verifyOscillates_cutsKpUntilCalm() {
        KpTuner[] tuner = new KpTuner[1];
        // Calm at every kP while tuning; once verifying, anything above half the tuned kP oscillates.
        tuner[0] = new KpTuner("test", KF, new double[]{30}, (target, kp) -> {
            boolean verifying = !Double.isNaN(tuner[0].getBestKp());
            return new KpTuner.Trial(target, kp, 500, 0, 0, verifying && kp > 0.5 * tuner[0].getAverageKp());
        });
        assertTrue(tuner[0].run());
        assertTrue(tuner[0].isVerified());
        assertEquals(tuner[0].getAverageKp() * Math.pow(0.8, 4), tuner[0].getBestKp(), 1e-12);
    }

    @Test
    public void cancel_stops() {
        KpTuner tuner = new KpTuner("test", KF, TARGETS, (target, kp) -> null);
        assertFalse(tuner.run());
        assertTrue(Double.isNaN(tuner.getBestKp()));
    }

    @Test
    public void analyze_flagsOscillationButNotOneOvershoot() {
        List<Double> t = new ArrayList<>(), ring = new ArrayList<>(), overshoot = new ArrayList<>();
        for (int i = 0; i < 200; i++) {
            double s = i * 0.02;
            t.add(s * 1000);
            ring.add(40 + 3 * Math.sin(2 * Math.PI * 2 * s));
            overshoot.add(40 - 10 * Math.exp(-3 * s) * Math.cos(5 * s));
        }
        assertTrue(KpTuner.analyze(40, 0.01, t, ring).oscillating);
        KpTuner.Trial one = KpTuner.analyze(40, 0.01, t, overshoot);
        assertFalse(one.oscillating);
        assertTrue(one.overshootRps > 0.75);
        assertTrue(one.settleMs > 0);
        assertFalse(KpTuner.analyze(40, 0.01, new ArrayList<>(), new ArrayList<>()).oscillating);
    }
}
