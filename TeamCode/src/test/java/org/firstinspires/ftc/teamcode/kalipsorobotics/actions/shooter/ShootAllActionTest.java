package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition.IntakeSide.A;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition.IntakeSide.B;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMechInterlockTest;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.ShooterRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Proxy;

/** The stopper must not open until shooter, turret AND both transfer sides are ready. */
public class ShootAllActionTest {
    private final double rps = IntakeConfig.FEED_RPS, tol = IntakeConfig.FEED_RPS_TOLERANCE;
    private final double timeout = IntakeConfig.FEED_GATE_TIMEOUT_MS, preFeed = IntakeConfig.PRE_FEED_MS;

    private final double[] rpsA = {0}, rpsB = {0}, powerA = {0}, powerB = {0};
    private boolean shooterInRange = true;
    private final Servo stopperServo = FlowerMechInterlockTest.fakeServo();
    private final Stopper stopper = new Stopper(new KServo(stopperServo, 1e6, 255, 0, false));
    private ShootAllAction shot;

    @Before
    public void setUp() throws Exception {
        IntakeConfig.FEED_RPS = 20;
        IntakeConfig.FEED_RPS_TOLERANCE = 1.5;
        IntakeConfig.FEED_GATE_TIMEOUT_MS = 60000;
        IntakeConfig.PRE_FEED_MS = 20;
        Intake intake = new Intake(new IntakeModule(A, motor(rpsA, powerA), new KServo(FlowerMechInterlockTest.fakeServo()), IntakeConfig.SIDE_A),
                new IntakeModule(B, motor(rpsB, powerB), new KServo(FlowerMechInterlockTest.fakeServo()), IntakeConfig.SIDE_B), null);
        Shooter shooter = fakeShooter();
        ShooterRun shooterRun = new ShooterRun(null, shooter, 30, 0.5) {
            @Override public void update() { }
            @Override public boolean isWithinRange() { return shooterInRange; }
        };
        TurretAutoAlign turret = new TurretAutoAlign(null, null) {
            @Override public boolean isWithinRange() { return true; }
            @Override public void stop() { }
        };
        shooterRun.setShooterRunMode(ShooterRunMode.SHOOT_USING_TARGET_RPS_HOOD);
        shot = new ShootAllAction(null, stopper, intake, shooter, shooterRun, turret, 30, 0.5, true);
        shot.setEnableLimelightZeroing(false);
    }

    @After
    public void restore() {
        IntakeConfig.FEED_RPS = rps;
        IntakeConfig.FEED_RPS_TOLERANCE = tol;
        IntakeConfig.FEED_GATE_TIMEOUT_MS = timeout;
        IntakeConfig.PRE_FEED_MS = preFeed;
    }

    @Test
    public void stopperStaysClosedUntilBothTransferSidesReachRps() throws Exception {
        stopper.open(); // left open by the driver: the shot must close it before spinning the transfer
        rpsA[0] = 20; // A at speed, B not
        run(200);
        assertEquals(ModuleConfig.STOPPER_SERVO_CLOSED_POS, stopperServo.getPosition(), 1e-9);
        assertFalse(shot.getPushBall().getHasStarted());
        assertTrue("transfer is being spun up", powerB[0] > 0);

        rpsB[0] = 19;
        run(200);
        assertEquals(ModuleConfig.STOPPER_SERVO_OPEN_POS, stopperServo.getPosition(), 1e-9);
        assertFalse(shot.transferTimedOut());
    }

    @Test
    public void transferHeldWhileShooterIsNotReady_andStoppedOnCancel() throws Exception {
        rpsA[0] = rpsB[0] = 20;
        shooterInRange = false;
        run(100);
        assertTrue(shot.getTransferReady().getIsDone());
        assertEquals(ModuleConfig.STOPPER_SERVO_CLOSED_POS, stopperServo.getPosition(), 1e-9);
        assertTrue(powerA[0] > 0 && powerB[0] > 0);
        shot.setIsDone(true);
        assertEquals(0, powerA[0], 1e-9);
        assertEquals(0, powerB[0], 1e-9);
    }

    private void run(long ms) throws InterruptedException {
        long end = System.currentTimeMillis() + ms;
        while (System.currentTimeMillis() < end) {
            shot.updateCheckDone();
            Thread.sleep(1);
        }
    }

    private static class FakeShooter extends Shooter {
        FakeShooter() { super(null); } // never called: needs a hardware map
        @Override public double getRPS() { return 30; }
    }

    /** Skips Shooter's constructor (no direct-injection one); Unsafe by reflection: android.jar does not have it. */
    private static Shooter fakeShooter() throws Exception {
        Class<?> unsafe = Class.forName("sun.misc.Unsafe");
        java.lang.reflect.Field f = unsafe.getDeclaredField("theUnsafe");
        f.setAccessible(true);
        return (Shooter) unsafe.getMethod("allocateInstance", Class.class).invoke(f.get(null), FakeShooter.class);
    }

    private static DcMotorEx motor(double[] rps, double[] power) {
        return (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(), new Class<?>[]{DcMotorEx.class},
                (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "setPower": power[0] = (double) args[0]; return null;
                        case "getPower": return power[0];
                        case "getVelocity": return rps[0] * ModuleConfig.INTAKE_TICKS_PER_REV;
                        default:
                            Class<?> r = method.getReturnType();
                            return r == boolean.class ? (Object) false : r == int.class ? (Object) 0 : r == double.class ? (Object) 0.0 : null;
                    }
                });
    }
}
