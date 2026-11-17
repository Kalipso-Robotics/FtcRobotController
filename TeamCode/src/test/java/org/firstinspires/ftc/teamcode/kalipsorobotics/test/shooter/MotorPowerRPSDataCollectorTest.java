package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class MotorPowerRPSDataCollectorTest {
    private boolean savedCompensation;

    @Before
    public void save() {
        savedCompensation = ShooterConfig.USE_VOLTAGE_COMPENSATION;
    }

    @After
    public void restore() {
        ShooterConfig.USE_VOLTAGE_COMPENSATION = savedCompensation;
    }

    @Test
    public void computeKf_withCompensation_scalesPowerByVoltageRatio() {
        ShooterConfig.USE_VOLTAGE_COMPENSATION = true;
        double v = ShooterInterpolationConfig.DEFAULT_VOLTAGE * 0.8;
        // effective power = 0.8 * 0.8 = 0.64, rps 32 -> 0.02
        assertEquals(0.02, MotorPowerRPSDataCollector.computeKf(0.8, v, 32), 1e-9);
    }

    @Test
    public void computeKf_withoutCompensation_isPowerOverRps() {
        ShooterConfig.USE_VOLTAGE_COMPENSATION = false;
        assertEquals(0.025, MotorPowerRPSDataCollector.computeKf(0.8, 10, 32), 1e-9);
    }

    @Test
    public void computeKf_zeroRps_isZero() {
        assertEquals(0, MotorPowerRPSDataCollector.computeKf(0.8, 12, 0), 0);
    }
}
