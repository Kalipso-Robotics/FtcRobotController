package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter.FlywheelPIDFTuner.RecoveryTest;
import org.junit.Test;

public class FlywheelPIDFTunerTest {
    @Test
    public void kfOnlyMode_forcesFeedbackGainsToZero() {
        org.junit.Assert.assertArrayEquals(new double[] {0, 0, 0}, FlywheelPIDFTuner.gains(true, 0.05, 0.01, 0.002), 0);
        org.junit.Assert.assertArrayEquals(new double[] {0.05, 0.01, 0.002}, FlywheelPIDFTuner.gains(false, 0.05, 0.01, 0.002), 0);
    }

    @Test
    public void recovery_dropsHoldsThenTimesTheClimbBack() {
        RecoveryTest test = new RecoveryTest();
        test.start(0);
        assertEquals(35, test.target(40, 5), 0);
        // at 35 for 300 ms -> recover phase starts at 300 ms, target back to 40
        for (double ms = 0; ms <= 300; ms += 20) {
            test.update(ms, 35, 40, 5);
        }
        assertEquals(RecoveryTest.Phase.RECOVER, test.phase);
        assertEquals(40, test.target(40, 5), 0);
        // 38 (outside the 2% band) until 400 ms, then 40.5 (1.25% over) until 500, then 40: recovered 100 ms after the jump
        for (double ms = 320; ms <= 1200 && test.phase != RecoveryTest.Phase.IDLE; ms += 20) {
            double rps = ms < 400 ? 38 : ms < 500 ? 40.5 : 40;
            test.update(ms, rps, 40, 5);
        }
        assertEquals(RecoveryTest.Phase.IDLE, test.phase);
        assertEquals("100 ms, overshoot 1.3%", test.result);
    }

    @Test
    public void recovery_timesOutWhenTheFlywheelNeverComesBack() {
        RecoveryTest test = new RecoveryTest();
        test.start(0);
        for (double ms = 0; ms <= 7000 && test.phase != RecoveryTest.Phase.IDLE; ms += 20) {
            test.update(ms, 35, 40, 5);
        }
        assertEquals("did not recover in 3 s (kF / kP too low?)", test.result);
    }
}
