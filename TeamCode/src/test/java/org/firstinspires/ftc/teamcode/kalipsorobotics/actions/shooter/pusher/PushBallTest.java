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
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.junit.After;
import org.junit.Test;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide.FRONT;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide.BACK;

public class PushBallTest {
    private final double preFeedMs = IntakeConfig.PRE_FEED_MS;
    private final double stopperOpenMs = IntakeConfig.STOPPER_OPEN_MS;
    private final double feedAMs = IntakeConfig.FEED_A_MS;
    private final double feedBMs = IntakeConfig.FEED_B_MS;
    private final double delayAMs = IntakeConfig.FEED_A_DELAY_MS;
    private final double delayBMs = IntakeConfig.FEED_B_DELAY_MS;
    private final double feedRps = IntakeConfig.FEED_RPS;
    private final double kfA = IntakeConfig.SIDE_A.kF, kfB = IntakeConfig.SIDE_B.kF;
    private final double kpA = IntakeConfig.SIDE_A.kP, kpB = IntakeConfig.SIDE_B.kP;

    @After
    public void restoreConfig() {
        IntakeConfig.PRE_FEED_MS = preFeedMs;
        IntakeConfig.STOPPER_OPEN_MS = stopperOpenMs;
        IntakeConfig.FEED_A_MS = feedAMs;
        IntakeConfig.FEED_B_MS = feedBMs;
        IntakeConfig.FEED_A_DELAY_MS = delayAMs;
        IntakeConfig.FEED_B_DELAY_MS = delayBMs;
        IntakeConfig.FEED_RPS = feedRps;
        IntakeConfig.SIDE_A.kF = kfA;
        IntakeConfig.SIDE_B.kF = kfB;
        IntakeConfig.SIDE_A.kP = kpA;
        IntakeConfig.SIDE_B.kP = kpB;
        IntakeConfig.FEED_RPS_COMP_ON = compOn;
        IntakeConfig.FEED_RPS_COMP_GAIN = compGain;
    }


    private final boolean compOn = IntakeConfig.FEED_RPS_COMP_ON;
    private final double compGain = IntakeConfig.FEED_RPS_COMP_GAIN;

    @Test
    public void rpsCompOn_shooterBelowTarget_feedsFaster_aboveTarget_feedsSlower() throws Exception {
        IntakeConfig.PRE_FEED_MS = 20;
        IntakeConfig.FEED_A_MS = 200;
        IntakeConfig.FEED_B_MS = 200;
        IntakeConfig.FEED_A_DELAY_MS = 0;
        IntakeConfig.FEED_B_DELAY_MS = 0;
        IntakeConfig.FEED_RPS = 10;
        IntakeConfig.SIDE_A.kF = 0.05; // fake encoders read 0 RPS: power = kF * target * voltageScale
        IntakeConfig.SIDE_B.kF = 0.03;
        IntakeConfig.SIDE_A.kP = 0;
        IntakeConfig.SIDE_B.kP = 0;
        IntakeConfig.FEED_RPS_COMP_ON = true;
        IntakeConfig.FEED_RPS_COMP_GAIN = 1;
        org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData.setVoltage(
                org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig.DEFAULT_VOLTAGE);
        double[] rpsError = {2}; // shooter 2 RPS under target -> transfer 12 RPS
        Rig rig = new Rig(() -> rpsError[0]);

        boolean sawFast = false, sawSlow = false;
        long deadline = System.currentTimeMillis() + 3000;
        while (!rig.pushBall.getIsDone()) {
            assertTrue("timed out", System.currentTimeMillis() < deadline);
            rig.pushBall.updateCheckDone();
            double a = rig.motorA.getPower(), b = rig.motorB.getPower();
            if (Math.abs(rig.stopperServo.getPosition() - ModuleConfig.STOPPER_SERVO_OPEN_POS) < 1e-9 && a != 0 && a != IntakeConfig.SHOOT_FEED_POWER) {
                if (rpsError[0] > 0) {
                    assertEquals(0.6, a, 1e-9);
                    assertEquals(0.36, b, 1e-9);
                    sawFast = true;
                    rpsError[0] = -3; // read live: shooter 3 RPS over target -> transfer 7 RPS
                } else {
                    assertEquals(0.35, a, 1e-9);
                    assertEquals(0.21, b, 1e-9);
                    sawSlow = true;
                }
            }
            Thread.sleep(1);
        }
        assertTrue(sawFast && sawSlow);
        rig.assertEndState();
    }

    @Test
    public void feedVelocityOn_preFeedOpenLoop_thenEachSideFeedsWithItsOwnGains() throws Exception {
        IntakeConfig.PRE_FEED_MS = 50;
        IntakeConfig.FEED_A_MS = 40;
        IntakeConfig.FEED_B_MS = 40;
        IntakeConfig.FEED_A_DELAY_MS = 0;
        IntakeConfig.FEED_B_DELAY_MS = 0;
        IntakeConfig.FEED_RPS = 10;
        IntakeConfig.SIDE_A.kF = 0.05; // fake encoders read 0 RPS: power = (kF + kP) * target * voltageScale
        IntakeConfig.SIDE_B.kF = 0.03;
        IntakeConfig.SIDE_A.kP = 0;
        IntakeConfig.SIDE_B.kP = 0;
        org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData.setVoltage(
                org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig.DEFAULT_VOLTAGE);
        Rig rig = new Rig();

        boolean sawFeed = false, opened = false;
        long deadline = System.currentTimeMillis() + 3000;
        while (!rig.pushBall.getIsDone()) {
            assertTrue("timed out", System.currentTimeMillis() < deadline);
            rig.pushBall.updateCheckDone();
            double a = rig.motorA.getPower(), b = rig.motorB.getPower();
            opened |= Math.abs(rig.stopperServo.getPosition() - ModuleConfig.STOPPER_SERVO_OPEN_POS) < 1e-9;
            if (!opened) {
                assertEquals("pre-feed A stays open loop", IntakeConfig.SHOOT_FEED_POWER, a, 1e-9);
                assertEquals("pre-feed B stays open loop", IntakeConfig.SHOOT_FEED_POWER, b, 1e-9);
            } else if (a != 0 && b != 0 && a != IntakeConfig.SHOOT_FEED_POWER) {
                assertEquals("feed A closed loop", 0.5, a, 1e-9);
                assertEquals("feed B closed loop", 0.3, b, 1e-9);
                sawFeed = true;
            }
            Thread.sleep(1);
        }
        assertTrue("both sides fed closed loop", sawFeed);
        rig.assertEndState();
    }

    @Test
    public void preFeedsBothSidesThenOpensAndFeedsWithoutAPowerGap() throws Exception {
        IntakeConfig.PRE_FEED_MS = 80;
        IntakeConfig.STOPPER_OPEN_MS = 100;
        IntakeConfig.FEED_A_MS = 60;
        IntakeConfig.FEED_B_MS = 60;
        IntakeConfig.FEED_A_DELAY_MS = 0;
        IntakeConfig.FEED_B_DELAY_MS = 0;
        Rig rig = new Rig();

        long startMs = -1, openMs = -1, stopMs = -1;
        long deadline = System.currentTimeMillis() + 3000;
        while (!rig.pushBall.getIsDone()) {
            assertTrue("timed out", System.currentTimeMillis() < deadline);
            rig.pushBall.updateCheckDone();
            double a = rig.motorA.getPower(), b = rig.motorB.getPower();
            long now = System.currentTimeMillis();
            boolean running = a != 0 && b != 0;
            if (startMs < 0) {
                assertTrue("both sides must spin from the first update", running);
                startMs = now;
            }
            if (openMs < 0 && Math.abs(rig.stopperServo.getPosition() - ModuleConfig.STOPPER_SERVO_OPEN_POS) < 1e-9) {
                openMs = now;
                assertTrue("both sides must still spin as the stopper opens", running);
            }
            if (a == 0 && b == 0 && stopMs < 0) {
                stopMs = now;
            }
            if (stopMs >= 0) {
                assertTrue("no power gap: once stopped, the feed is over", a == 0 && b == 0);
            }
            Thread.sleep(1);
        }
        assertTrue("stopper must stay closed for the pre-feed", openMs >= 0 && openMs - startMs >= 70);
        // open command -> KServo estimate (~54ms) -> STOPPER_OPEN_MS (100) -> FEED_A/B_MS (60)
        assertTrue("the feed time counts from the stopper being open", stopMs - openMs >= 100 + 60);
        rig.assertEndState();
    }

    @Test
    public void feedBDelay_startsBAfterA() throws Exception {
        IntakeConfig.PRE_FEED_MS = 50;
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
                assertEquals("a delayed side skips the pre-feed", ModuleConfig.STOPPER_SERVO_OPEN_POS, rig.stopperServo.getPosition(), 1e-9);
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
            this(null);
        }

        Rig(java.util.function.DoubleSupplier shooterRpsError) {
            IntakeModule sideA = new IntakeModule(FRONT, motorA, new KServo(FlowerMechInterlockTest.fakeServo(), 1e6, 255, 0, false), IntakeConfig.SIDE_A);
            IntakeModule sideB = new IntakeModule(BACK, motorB, new KServo(FlowerMechInterlockTest.fakeServo(), 1e6, 255, 0, false), IntakeConfig.SIDE_B);
            FlowerMech flowerMech = new FlowerMech(new KServo(FlowerMechInterlockTest.fakeServo(), 1e6, 255, 0, false), sideA);
            Intake intake = new Intake(sideA, sideB, flowerMech);
            Stopper stopper = new Stopper(new KServo(stopperServo, 1e6, 255, 0, false));
            pushBall = new PushBall(stopper, intake, shooterRpsError);
        }

        void assertEndState() {
            assertEquals(0, motorA.getPower(), 1e-9);
            assertEquals(0, motorB.getPower(), 1e-9);
            assertEquals(ModuleConfig.STOPPER_SERVO_CLOSED_POS, stopperServo.getPosition(), 1e-9);
        }
    }
}
