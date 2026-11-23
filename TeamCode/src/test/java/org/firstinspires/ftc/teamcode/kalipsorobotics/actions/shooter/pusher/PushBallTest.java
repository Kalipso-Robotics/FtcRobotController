package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMechInterlockTest;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KCRServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.junit.After;
import org.junit.Test;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition.IntakeSide.A;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition.IntakeSide.B;

public class PushBallTest {
    private final double feedAMs = IntakeConfig.FEED_A_MS;
    private final double feedBMs = IntakeConfig.FEED_B_MS;
    private final double delayAMs = IntakeConfig.FEED_A_DELAY_MS;
    private final double delayBMs = IntakeConfig.FEED_B_DELAY_MS;

    @After
    public void restoreConfig() {
        IntakeConfig.FEED_A_MS = feedAMs;
        IntakeConfig.FEED_B_MS = feedBMs;
        IntakeConfig.FEED_A_DELAY_MS = delayAMs;
        IntakeConfig.FEED_B_DELAY_MS = delayBMs;
    }

    @Test
    public void feedsBothSidesAtOnceWithStopperOpenAndClosesAtEnd() throws Exception {
        IntakeConfig.FEED_A_MS = 60;
        IntakeConfig.FEED_B_MS = 60;
        IntakeConfig.FEED_A_DELAY_MS = 0;
        IntakeConfig.FEED_B_DELAY_MS = 0;
        Rig rig = new Rig();

        boolean sawBoth = false;
        long deadline = System.currentTimeMillis() + 3000;
        while (!rig.pushBall.getIsDone()) {
            assertTrue("timed out", System.currentTimeMillis() < deadline);
            rig.pushBall.updateCheckDone();
            double a = rig.motorA.getPower(), b = rig.motorB.getPower();
            if (a != 0 || b != 0) {
                assertEquals("stopper must be open before any feed", ModuleConfig.STOPPER_SERVO_OPEN_POS, rig.stopperServo.getPosition(), 1e-9);
            }
            if (a != 0 && b != 0) {
                sawBoth = true;
            }
            Thread.sleep(1);
        }
        assertTrue("A and B must feed at the same time", sawBoth);
        rig.assertEndState();
    }

    @Test
    public void feedBDelay_startsBAfterA() throws Exception {
        IntakeConfig.FEED_A_MS = 40;
        IntakeConfig.FEED_B_MS = 40;
        IntakeConfig.FEED_A_DELAY_MS = 0;
        IntakeConfig.FEED_B_DELAY_MS = 120;
        Rig rig = new Rig();

        boolean sawAOnly = false, sawB = false;
        long deadline = System.currentTimeMillis() + 3000;
        while (!rig.pushBall.getIsDone()) {
            assertTrue("timed out", System.currentTimeMillis() < deadline);
            rig.pushBall.updateCheckDone();
            double a = rig.motorA.getPower(), b = rig.motorB.getPower();
            if (a != 0 && b == 0 && !sawB) {
                sawAOnly = true;
            }
            if (b != 0) {
                assertTrue("B must wait for its delay", sawAOnly);
                sawB = true;
            }
            Thread.sleep(1);
        }
        assertTrue(sawAOnly && sawB);
        rig.assertEndState();
    }

    @Test
    public void hasStarted_trueAfterFirstUnblockedUpdate() {
        Rig rig = new Rig();
        assertTrue(!rig.pushBall.getHasStarted());
        rig.pushBall.updateCheckDone();
        assertTrue("RedDecideAuto only drops a shoot before its feed starts", rig.pushBall.getHasStarted());
    }

    private static class Rig {
        final Servo stopperServo = FlowerMechInterlockTest.fakeServo();
        final DcMotor motorA = FlowerMechInterlockTest.fakeMotor();
        final DcMotor motorB = FlowerMechInterlockTest.fakeMotor();
        final PushBall pushBall;

        Rig() {
            IntakeSide sideA = new IntakeSide(A, motorA, new KServo(FlowerMechInterlockTest.fakeServo(), 1e6, 255, 0, false));
            IntakeSide sideB = new IntakeSide(B, motorB, new KServo(FlowerMechInterlockTest.fakeServo(), 1e6, 255, 0, false));
            FlowerMech flowerMech = new FlowerMech(new KCRServo(FlowerMechInterlockTest.fakeCRServo(), false), sideA);
            Intake intake = new Intake(sideA, sideB, flowerMech);
            Stopper stopper = new Stopper(new KServo(stopperServo, 1e6, 255, 0, false));
            pushBall = new PushBall(stopper, intake);
        }

        void assertEndState() {
            assertEquals(0, motorA.getPower(), 1e-9);
            assertEquals(0, motorB.getPower(), 1e-9);
            assertEquals(ModuleConfig.STOPPER_SERVO_CLOSED_POS, stopperServo.getPosition(), 1e-9);
        }
    }
}
