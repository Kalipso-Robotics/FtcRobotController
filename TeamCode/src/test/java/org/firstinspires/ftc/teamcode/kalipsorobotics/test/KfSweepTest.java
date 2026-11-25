package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.junit.Test;

public class KfSweepTest {
    @Test
    public void averagesKfPerStepThenOverSteps_skipsAMotorThatNeverMoves() throws Exception {
        double[] powers = {0.4, 0.8};
        KfSweep sweep = new KfSweep("test", powers, 60, 20, "spins", "stalled");
        double voltage = ShooterInterpolationConfig.DEFAULT_VOLTAGE; // voltageScale = 1
        sweep.start();
        long deadline = System.currentTimeMillis() + 2000;
        boolean done = false;
        while (!done) {
            assertTrue("timed out", System.currentTimeMillis() < deadline);
            double p = sweep.power();
            // "spins": kF 0.04 at 0.4 power, 0.05 at 0.8 power -> result 0.045. "stalled": never moves.
            double rps = p == 0.4 ? 10 : 16;
            done = sweep.update(new double[]{rps, 0}, voltage);
            Thread.sleep(1);
        }
        assertFalse(sweep.isRunning());
        assertEquals(0, sweep.power(), 0);
        assertEquals(2, sweep.doneSteps());
        assertEquals(0.04, sweep.stepKf(0, 0), 1e-9);
        assertEquals(0.05, sweep.stepKf(0, 1), 1e-9);
        assertEquals(0.045, sweep.result(0), 1e-9);
        assertTrue(Double.isNaN(sweep.result(1)));
    }

    @Test
    public void cancel_stopsWithoutAResult() {
        KfSweep sweep = new KfSweep("test", new double[]{0.5}, 60, 20, "m");
        sweep.start();
        sweep.cancel();
        assertFalse(sweep.isRunning());
        assertEquals(0, sweep.power(), 0);
        assertTrue(Double.isNaN(sweep.result(0)));
    }
}
