package org.firstinspires.ftc.teamcode.kalipsorobotics.utilities;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMechInterlockTest;
import org.junit.Test;

public class KServoLimitsTest {
    @Test
    public void clampsToHardLimits() {
        KServo s = new KServo(FlowerMechInterlockTest.fakeServo()).setLimits(0.2, 0.6);
        s.setPosition(0.9);
        assertEquals(0.6, s.getPosition(), 1e-9);
        s.setTargetPosition(0.05);
        assertEquals(0.2, s.getPosition(), 1e-9);
        s.getServo().setPosition(0.9); // raw servo bypasses limits (HoodServoToZero)
        assertEquals(0.9, s.getPosition(), 1e-9);
    }
}
