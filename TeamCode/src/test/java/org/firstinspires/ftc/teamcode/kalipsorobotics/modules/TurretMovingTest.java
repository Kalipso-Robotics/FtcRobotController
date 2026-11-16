package org.firstinspires.ftc.teamcode.kalipsorobotics.modules;

import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerInterlockTest;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.junit.Test;

/** isMoving() must cover the whole of a big swing, even when a tiny re-target follows it right away. */
public class TurretMovingTest {

    @Test
    public void tinyRetargetRightAfterBigSwing_isStillMoving() throws InterruptedException {
        // 400 deg/s: a 0 -> MAX swing (~134 servo deg) takes a few hundred ms.
        Turret turret = new Turret(new KServo(FlowerInterlockTest.fakeServo(), 400,
                TurretConfig.SERVO_RANGE_DEG, TurretConfig.SERVO_ZERO_POS, false));
        turret.setAngleDeg(TurretConfig.MAX_TURRET_DEG);
        turret.setAngleDeg(TurretConfig.MAX_TURRET_DEG - 0.5);
        Thread.sleep(30);
        assertTrue("turret must still be travelling the big swing", turret.isMoving());
    }
}
