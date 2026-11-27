package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide.FRONT;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide.BACK;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMechInterlockTest;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Proxy;

/** The one-line auto intake actions: lower/raise/stow a side, the Flower arm, and the lowerFirst runs. */
public class IntakeActionsTest {
    private final double pivotMs = IntakeConfig.PIVOT_MOVE_MS;
    private final double kfA = IntakeConfig.SIDE_A.kF, kfB = IntakeConfig.SIDE_B.kF;
    private final double kpA = IntakeConfig.SIDE_A.kP, kpB = IntakeConfig.SIDE_B.kP;
    private final double voltage = SharedData.getVoltage();

    private final Servo pivotServoA = FlowerMechInterlockTest.fakeServo();
    private final Servo pivotServoB = FlowerMechInterlockTest.fakeServo();
    private final Servo flowerServo = FlowerMechInterlockTest.fakeServo();
    private final DcMotor motorA = FlowerMechInterlockTest.fakeMotor();
    private final DcMotor motorB = FlowerMechInterlockTest.fakeMotor();
    private IntakeModule sideA;
    private IntakeModule sideB;
    private FlowerMech flowerMech;
    private Intake intake;

    @Before
    public void setUp() {
        IntakeConfig.PIVOT_MOVE_MS = 20;
        // Feed RPS loop (IntakeModule.runRPS): different gains per side so a side mix-up fails.
        SharedData.setVoltage(ShooterInterpolationConfig.DEFAULT_VOLTAGE); // voltageScale = 1
        IntakeConfig.SIDE_A.kF = 0.05;
        IntakeConfig.SIDE_B.kF = 0.08;
        IntakeConfig.SIDE_A.kP = 0;
        IntakeConfig.SIDE_B.kP = 0;
        sideA = new IntakeModule(FRONT, motorA, new KServo(pivotServoA, 1e6, 255, 0, false), IntakeConfig.SIDE_A);
        sideB = new IntakeModule(BACK, motorB, new KServo(pivotServoB, 1e6, 255, 0, false), IntakeConfig.SIDE_B);
        flowerMech = new FlowerMech(new KServo(flowerServo, 1e6, 255, 0, false), sideA);
        intake = new Intake(sideA, sideB, flowerMech);
    }

    @After
    public void restoreConfig() {
        IntakeConfig.PIVOT_MOVE_MS = pivotMs;
        IntakeConfig.SIDE_A.kF = kfA;
        IntakeConfig.SIDE_B.kF = kfB;
        IntakeConfig.SIDE_A.kP = kpA;
        IntakeConfig.SIDE_B.kP = kpB;
        SharedData.setVoltage(voltage);
    }

    // ---- Feed RPS loop, per side (IntakeModule.runRPS / getRPS / estimateKf) ----

    @Test
    public void atTarget_powerIsKfTimesTarget_perSide() {
        VelocityRig a = new VelocityRig(IntakeSide.FRONT, 10);
        VelocityRig b = new VelocityRig(IntakeSide.BACK, 10);
        a.side.runRPS(10);
        b.side.runRPS(10);
        assertEquals(0.5, a.power(), 1e-9);
        assertEquals(0.8, b.power(), 1e-9);
    }

    @Test
    public void slowedByLoad_kpAddsPower_onlyOnThatSide() {
        IntakeConfig.SIDE_A.kP = 0.1;
        VelocityRig a = new VelocityRig(IntakeSide.FRONT, 8); // 2 RPS under target
        VelocityRig b = new VelocityRig(IntakeSide.BACK, 8);
        a.side.runRPS(10);
        b.side.runRPS(10);
        assertEquals("A: kF*target + kP*error", 0.5 + 0.2, a.power(), 1e-9);
        assertEquals("B has kP = 0: feedforward only", 0.8, b.power(), 1e-9);
    }

    @Test
    public void lowBattery_scalesPowerUp() {
        SharedData.setVoltage(ShooterInterpolationConfig.DEFAULT_VOLTAGE * 0.9);
        VelocityRig a = new VelocityRig(IntakeSide.FRONT, 10);
        a.side.runRPS(10);
        assertTrue("sagging battery needs more power", a.power() > 0.5);
    }

    @Test
    public void powerClampedToOne_andRunGoesBackToOpenLoop() {
        IntakeConfig.SIDE_B.kP = 1;
        VelocityRig b = new VelocityRig(IntakeSide.BACK, 0); // stalled
        b.side.runRPS(10);
        assertEquals(1, b.power(), 1e-9);
        b.side.run(0.3);
        assertEquals(0.3, b.power(), 1e-9);
    }

    @Test
    public void estimateKf_isPowerOverSpeed_perSide() {
        VelocityRig a = new VelocityRig(IntakeSide.FRONT, 18);
        VelocityRig b = new VelocityRig(IntakeSide.BACK, 15);
        a.side.run(0.9);
        b.side.run(0.9);
        assertEquals(0.05, a.side.estimateKf(), 1e-9);
        assertEquals(0.06, b.side.estimateKf(), 1e-9);
        assertTrue("not moving = no estimate", Double.isNaN(new VelocityRig(IntakeSide.FRONT, 0).side.estimateKf()));
    }

    @Test
    public void intakeReady_doneWhenEitherSideInTolerance_elseTimesOut() throws Exception {
        double tol = IntakeConfig.FEED_RPS_TOLERANCE, timeout = IntakeConfig.FEED_GATE_TIMEOUT_MS;
        try {
            IntakeConfig.FEED_RPS_TOLERANCE = 1.5;
            IntakeConfig.FEED_GATE_TIMEOUT_MS = 100;
            IntakeRun run = new IntakeRun(new Intake(new VelocityRig(FRONT, 10).side, new VelocityRig(BACK, 21.4).side, null), 20);
            IntakeReady ready = new IntakeReady(run);
            run.updateCheckDone();
            ready.updateCheckDone();
            assertTrue("B in tolerance is enough", ready.getIsDone());
            assertFalse(ready.timedOut());

            VelocityRig slowB = new VelocityRig(BACK, 17);
            IntakeRun stuckRun = new IntakeRun(new Intake(new VelocityRig(FRONT, 17).side, slowB.side, null), 20);
            IntakeReady stuck = new IntakeReady(stuckRun);
            stuckRun.updateCheckDone();
            stuck.updateCheckDone();
            assertFalse("both out of tolerance: keeps waiting", stuck.getIsDone());
            assertTrue("IntakeRun holds the RPS while waiting", slowB.power() > 0);
            Thread.sleep(120);
            stuckRun.updateCheckDone();
            stuck.updateCheckDone();
            assertTrue(stuck.getIsDone());
            assertTrue(stuck.timedOut());

            stuckRun.stop();
            assertTrue("stop() finishes IntakeRun", stuckRun.getIsDone());
            assertEquals(0, slowB.power(), 1e-9);
        } finally {
            IntakeConfig.FEED_RPS_TOLERANCE = tol;
            IntakeConfig.FEED_GATE_TIMEOUT_MS = timeout;
        }
    }

    /** An IntakeModule on a fake DcMotorEx whose encoder always reads the given RPS (as ticks/s, like the hub). */
    private static class VelocityRig {
        final double[] power = {0};
        final IntakeModule side;

        VelocityRig(IntakeSide which, double rps) {
            double ticksPerSec = rps * ModuleConfig.INTAKE_TICKS_PER_REV;
            DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(), new Class<?>[]{DcMotorEx.class},
                    (proxy, method, args) -> {
                        switch (method.getName()) {
                            case "setPower": power[0] = (double) args[0]; return null;
                            case "getPower": return power[0];
                            case "getVelocity": return ticksPerSec;
                            default:
                                Class<?> r = method.getReturnType();
                                if (r == boolean.class) return false;
                                if (r == int.class) return 0;
                                if (r == double.class) return 0.0;
                                return null;
                        }
                    });
            side = new IntakeModule(which, motor, new KServo(FlowerMechInterlockTest.fakeServo()), (which == IntakeSide.FRONT ? IntakeConfig.SIDE_A : IntakeConfig.SIDE_B));
        }

        double power() {
            return power[0];
        }
    }

    @Test
    public void sideLower_lowersAndWaitsPivotMove() throws Exception {
        long start = System.currentTimeMillis();
        runToDone(new IntakeModuleLower(sideA));
        assertTrue(System.currentTimeMillis() - start >= IntakeConfig.PIVOT_MOVE_MS);
        assertEquals(IntakeConfig.SIDE_A.pivotDownPos, pivotServoA.getPosition(), 1e-9);
        assertTrue(sideA.isDown());
    }

    @Test
    public void sideRaise_raisesFlowerFirstThenSide() throws Exception {
        runToDone(new IntakeModuleLower(sideA));
        flowerMech.getServo().setTargetPosition(FlowerMech.DOWN_POS);
        runToDone(new IntakeModuleRaise(sideA));
        assertEquals(FlowerMech.UP_POS, flowerServo.getPosition(), 1e-9);
        assertEquals(IntakeConfig.SIDE_A.pivotUpPos, pivotServoA.getPosition(), 1e-9);
        assertTrue(sideA.isUp());
    }

    @Test
    public void stow_stopsAndRaisesEverything() throws Exception {
        runToDone(new IntakeModuleLower(sideA));
        runToDone(new IntakeModuleLower(sideB));
        flowerMech.getServo().setTargetPosition(FlowerMech.DOWN_POS);
        intake.run(1);
        runToDone(new IntakeStow(intake));
        assertEquals(0, motorA.getPower(), 1e-9);
        assertEquals(0, motorB.getPower(), 1e-9);
        assertEquals(FlowerMech.UP_POS, flowerServo.getPosition(), 1e-9);
        assertEquals(IntakeConfig.SIDE_A.pivotUpPos, pivotServoA.getPosition(), 1e-9);
        assertEquals(IntakeConfig.SIDE_B.pivotUpPos, pivotServoB.getPosition(), 1e-9);
    }

    /** PushBall and auto run one side with +power, so run() must not negate a side (direction lives in ModuleConfig). */
    @Test
    public void runBoth_samePowerAsPerSideRun() {
        intake.run(1);
        assertEquals(1, motorA.getPower(), 1e-9);
        assertEquals(1, motorB.getPower(), 1e-9);
    }

    @Test
    public void flowerLower_thenRaise_onceSideIsDown() throws Exception {
        runToDone(new IntakeModuleLower(sideA));
        runToDone(new KServoAutoAction(flowerMech.getServo(), FlowerMech.DOWN_POS));
        assertTrue(flowerMech.isDown());
        assertEquals(FlowerMech.DOWN_POS, flowerServo.getPosition(), 1e-9);
        runToDone(new KServoAutoAction(flowerMech.getServo(), FlowerMech.UP_POS));
        assertTrue(flowerMech.isUp());
        assertEquals(FlowerMech.UP_POS, flowerServo.getPosition(), 1e-9);
    }

    @Test
    public void runIntakeTime_lowerFirst_lowersSide_threeArgFormDoesNot() throws Exception {
        runToDone(new RunIntakeTime(sideB, 10, 1));
        assertEquals(IntakeConfig.SIDE_B.pivotUpPos, pivotServoB.getPosition(), 1e-9);
        runToDone(new RunIntakeTime(sideB, 10, 1, true));
        assertEquals(IntakeConfig.SIDE_B.pivotDownPos, pivotServoB.getPosition(), 1e-9);
        assertEquals(0, motorB.getPower(), 1e-9);
    }

    @Test
    public void intakeFullAction_oneSide_lowersItsSide() throws Exception {
        Stopper stopper = new Stopper(new KServo(FlowerMechInterlockTest.fakeServo(), 1e6, 255, 0, false));
        runToDone(new IntakeFullAction(stopper, sideB, 10, 1));
        assertEquals(IntakeConfig.SIDE_B.pivotDownPos, pivotServoB.getPosition(), 1e-9);
    }

    @Test
    public void driveDirectionIntakeAction_setDone_stopsAndRaisesBoth() {
        runToDone(new IntakeModuleLower(sideA));
        runToDone(new IntakeModuleLower(sideB));
        intake.run(1);
        DriveDirectionIntakeAction action = new DriveDirectionIntakeAction(intake);
        action.setIsDone(true);
        assertEquals(0, motorA.getPower(), 1e-9);
        assertEquals(0, motorB.getPower(), 1e-9);
        assertEquals(IntakeConfig.SIDE_A.pivotUpPos, pivotServoA.getPosition(), 1e-9);
        assertEquals(IntakeConfig.SIDE_B.pivotUpPos, pivotServoB.getPosition(), 1e-9);
        assertFalse(sideA.isDown());
    }

    private static void runToDone(Action action) {
        long deadline = System.currentTimeMillis() + 3000;
        while (!action.updateCheckDone()) {
            assertTrue("timed out: " + action.getName(), System.currentTimeMillis() < deadline);
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
