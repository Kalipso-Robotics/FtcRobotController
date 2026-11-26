package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Reads the thresholds from IntakeConfig, so retuning them does not break these. */
public class IntakeFullDetectorTest {
    private static final double HIGH_MA = IntakeConfig.STALL_CURRENT_MA + 1000;
    private static final double LOW_MA = IntakeConfig.BALL_1_MA - 100;
    private static final double STALLED_RPS = 0;
    private static final double SPINNING_RPS = IntakeConfig.STALL_RPS + 5;
    private static final long IGNORE = (long) IntakeConfig.STALL_IGNORE_MS;
    private static final long CONFIRM = (long) IntakeConfig.STALL_CONFIRM_MS;

    @Test
    public void spinUpSpikeIsIgnored() {
        IntakeFullDetector d = new IntakeFullDetector();
        d.reset(0);
        for (long t = 0; t < IGNORE; t += 20) {
            d.update(9000, STALLED_RPS, t);
        }
        assertFalse(d.isFull());
        assertEquals(0, d.estimateBalls());
    }

    @Test
    public void shortStallIsNotFull_sustainedStallIsAndStaysFull() {
        IntakeFullDetector d = new IntakeFullDetector();
        d.reset(0);
        long t = IGNORE;
        for (; t < IGNORE + CONFIRM / 2; t += 20) {
            d.update(HIGH_MA, STALLED_RPS, t); // one ball jamming briefly
        }
        for (; t < IGNORE + 2 * CONFIRM; t += 20) {
            d.update(LOW_MA, SPINNING_RPS, t);
        }
        assertFalse(d.isFull());

        long stallStart = t;
        for (; t < stallStart + 3 * CONFIRM; t += 20) {
            d.update(HIGH_MA, STALLED_RPS, t);
        }
        assertTrue(d.isFull());
        d.update(0, 0, t + 20); // motor stopped: the balls are still in
        assertTrue(d.isFull());
        assertEquals(IntakeFullDetector.CAPACITY, d.estimateBalls());
        d.reset(t + 40);
        assertFalse(d.isFull());
    }

    @Test
    public void highCurrentWhileStillSpinningIsNotFull() {
        IntakeFullDetector d = new IntakeFullDetector();
        d.reset(0);
        for (long t = IGNORE; t < IGNORE + 5 * CONFIRM; t += 20) {
            d.update(HIGH_MA, SPINNING_RPS, t); // working hard but balls still moving through
        }
        assertFalse(d.isFull());
    }

    @Test
    public void estimateCountsBallThresholds() {
        double[] levels = {LOW_MA, IntakeConfig.BALL_1_MA, IntakeConfig.BALL_2_MA, IntakeConfig.BALL_3_MA, IntakeConfig.BALL_4_MA};
        for (int balls = 0; balls < levels.length; balls++) {
            IntakeFullDetector d = new IntakeFullDetector();
            d.reset(0);
            d.update(levels[balls], SPINNING_RPS, IGNORE);
            assertEquals(balls, d.estimateBalls());
        }
    }
}
