package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Uses the class defaults: empty 1500 mA, full 4000 mA, hold 250 ms, ignore the first 300 ms. */
public class IntakeFullDetectorTest {

    @Test
    public void spinUpSpikeIsIgnored() {
        IntakeFullDetector d = new IntakeFullDetector();
        d.reset(0);
        for (long t = 0; t < 300; t += 20) {
            d.update(9000, t);
        }
        assertFalse(d.isFull());
        assertEquals(0, d.estimateBalls());
    }

    @Test
    public void shortBumpIsNotFull_sustainedHighCurrentIsAndStaysFull() {
        IntakeFullDetector d = new IntakeFullDetector();
        d.reset(0);
        long t = 300;
        for (; t < 400; t += 20) {
            d.update(5000, t); // one ball bumping through, 100 ms
        }
        for (; t < 600; t += 20) {
            d.update(1500, t);
        }
        assertFalse(d.isFull());

        for (; t < 1200; t += 20) {
            d.update(5000, t);
        }
        assertTrue(d.isFull());
        d.update(1500, t + 20); // robot stopped pushing: the balls are still in
        assertTrue(d.isFull());
        assertEquals(4, d.estimateBalls());
    }

    @Test
    public void estimateIsBetweenEmptyAndFull() {
        IntakeFullDetector d = new IntakeFullDetector();
        d.reset(0);
        d.update(1500, 300);
        assertEquals(0, d.estimateBalls());
        d.reset(0);
        d.update(2750, 300); // halfway
        assertEquals(2, d.estimateBalls());
    }
}
