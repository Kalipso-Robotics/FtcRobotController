package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.stopper;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher.CloseStopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerInterlockTest;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.junit.Test;

public class StopperActionsTest {
    private final Servo servo = FlowerInterlockTest.fakeServo();
    private final Stopper stopper = new Stopper(new KServo(servo, 1e6, 255, 0, false));

    @Test
    public void openStopperAction_opens() throws Exception {
        OpenStopperAction open = new OpenStopperAction(stopper);
        long deadline = System.currentTimeMillis() + 2000;
        while (!open.updateCheckDone()) {
            assertTrue(System.currentTimeMillis() < deadline);
            Thread.sleep(1);
        }
        assertEquals(ModuleConfig.STOPPER_SERVO_OPEN_POS, servo.getPosition(), 1e-9);
    }

    @Test
    public void closeInstantDone_keepsKServoTargetInSync() {
        stopper.open();
        CloseStopper close = new CloseStopper(stopper);
        assertTrue(close.updateCheckDone());
        assertEquals(ModuleConfig.STOPPER_SERVO_CLOSED_POS, stopper.getStopper().getTargetPosition(), 1e-9);
    }
}
