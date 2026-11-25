package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KMotor;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class MotorPowerRPSDataCollectorTest {
    private double savedExponent;

    @Before
    public void save() {
        savedExponent = ShooterConfig.VOLTAGE_COMP_EXPONENT;
    }

    @After
    public void restore() {
        ShooterConfig.VOLTAGE_COMP_EXPONENT = savedExponent;
    }

    @Test
    public void computeKf_withCompensation_scalesPowerByVoltageRatio() {
        ShooterConfig.VOLTAGE_COMP_EXPONENT = 1;
        double v = ShooterInterpolationConfig.DEFAULT_VOLTAGE * 0.8;
        // effective power = 0.8 * 0.8 = 0.64, rps 32 -> 0.02
        assertEquals(0.02, MotorPowerRPSDataCollector.computeKf(0.8, v, 32), 1e-9);
    }

    @Test
    public void computeKf_withoutCompensation_isPowerOverRps() {
        ShooterConfig.VOLTAGE_COMP_EXPONENT = 0;
        assertEquals(0.025, MotorPowerRPSDataCollector.computeKf(0.8, 10, 32), 1e-9);
    }

    @Test
    public void computeKf_zeroRps_isZero() {
        assertEquals(0, MotorPowerRPSDataCollector.computeKf(0.8, 12, 0), 0);
    }

    /** The kf measured at any voltage gives back the same motor power through goToRPS's scale. */
    @Test
    public void computeKf_undoesTheVoltageScaleAtAPartialExponent() {
        ShooterConfig.VOLTAGE_COMP_EXPONENT = 0.6;
        double kf = MotorPowerRPSDataCollector.computeKf(0.55, 11.8, 41);
        assertEquals(0.55, kf * 41 * KMotor.voltageScale(11.8), 1e-9);
        assertEquals(1.0357, KMotor.voltageScale(11.787), 1e-3);
        assertEquals(1, KMotor.voltageScale(ShooterInterpolationConfig.DEFAULT_VOLTAGE), 1e-12);
        assertEquals(1, KMotor.voltageScale(0), 0); // no voltage read yet
    }
}
