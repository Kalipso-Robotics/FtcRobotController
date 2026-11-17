package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Flower;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerInterlockTest;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.junit.Test;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition.IntakeSide.A;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition.IntakeSide.B;

public class PushBallTest {

    @Test
    public void feedsAThenBWithStopperOpenAndClosesAtEnd() throws Exception {
        IntakeConfig.FEED_A_MS = 30;
        IntakeConfig.WAIT_BETWEEN_SIDES_MS = 30;
        IntakeConfig.FEED_B_MS = 30;

        Servo stopperServo = FlowerInterlockTest.fakeServo();
        DcMotor motorA = FlowerInterlockTest.fakeMotor();
        DcMotor motorB = FlowerInterlockTest.fakeMotor();
        IntakeSide sideA = new IntakeSide(A, motorA, new KServo(FlowerInterlockTest.fakeServo(), 1e6, 255, 0, false));
        IntakeSide sideB = new IntakeSide(B, motorB, new KServo(FlowerInterlockTest.fakeServo(), 1e6, 255, 0, false));
        Flower flower = new Flower(new KServo(FlowerInterlockTest.fakeServo(), 1e6, 255, 0, false), sideA);
        Intake intake = new Intake(sideA, sideB, flower);
        Stopper stopper = new Stopper(new KServo(stopperServo, 1e6, 255, 0, false));

        PushBall pushBall = new PushBall(stopper, intake);
        boolean sawA = false, sawGap = false, sawB = false;
        long deadline = System.currentTimeMillis() + 3000;
        while (!pushBall.getIsDone()) {
            assertTrue("timed out", System.currentTimeMillis() < deadline);
            pushBall.updateCheckDone();
            double a = motorA.getPower(), b = motorB.getPower();
            if (a != 0 || b != 0) {
                assertEquals("stopper must be open before any feed", ModuleConfig.STOPPER_SERVO_OPEN_POS, stopperServo.getPosition(), 1e-9);
            }
            if (a != 0) {
                assertEquals(0, b, 1e-9);
                assertTrue("A must run before B", !sawB);
                sawA = true;
            }
            if (b != 0) {
                assertEquals(0, a, 1e-9);
                assertTrue("B must follow A", sawA);
                sawB = true;
            }
            if (sawA && !sawB && a == 0 && b == 0) {
                sawGap = true;
            }
            Thread.sleep(1);
        }
        assertTrue(sawA && sawGap && sawB);
        assertEquals(0, motorA.getPower(), 1e-9);
        assertEquals(0, motorB.getPower(), 1e-9);
        assertEquals(ModuleConfig.STOPPER_SERVO_CLOSED_POS, stopperServo.getPosition(), 1e-9);
    }
}
