package org.firstinspires.ftc.teamcode.kalipsorobotics.modules;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.junit.Test;

public class TurretTest {

    @Test
    public void gearRatio_matchesCad() {
        assertEquals(1.2065, TurretConfig.TURRET_DEG_PER_SERVO_DEG, 1e-4);
    }

    @Test
    public void straightAhead_isZeroPosition() {
        assertEquals(TurretConfig.SERVO_ZERO_POS, Turret.servoPosFromTurretDeg(0), 1e-12);
        assertEquals(0, Turret.turretDegFromServoPos(TurretConfig.SERVO_ZERO_POS), 1e-12);
    }

    @Test
    public void degToPosToDeg_roundTripsAcrossReachableRange() {
        for (double deg = TurretConfig.MIN_TURRET_DEG; deg <= TurretConfig.MAX_TURRET_DEG; deg += 7.5) {
            assertEquals(deg, Turret.turretDegFromServoPos(Turret.servoPosFromTurretDeg(deg)), 1e-9);
        }
    }

    @Test
    public void targetBeyondRange_clampsToLimit() {
        assertEquals(TurretConfig.MAX_TURRET_DEG, Turret.clampTurretDeg(TurretConfig.MAX_TURRET_DEG + 15), 1e-12);
        assertEquals(TurretConfig.MIN_TURRET_DEG, Turret.clampTurretDeg(TurretConfig.MIN_TURRET_DEG - 15), 1e-12);
        assertEquals(12.4, Turret.clampTurretDeg(12.4), 1e-12);
    }

    @Test
    public void servoPos_staysInsideZeroToOne() {
        for (double deg = -360; deg <= 360; deg += 5) {
            double pos = Turret.servoPosFromTurretDeg(Turret.clampTurretDeg(deg));
            assertTrue("deg=" + deg + " pos=" + pos, pos >= 0 && pos <= 1);
        }
    }
}
