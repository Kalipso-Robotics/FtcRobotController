package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition.IntakeSide.A;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition.IntakeSide.B;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMechInterlockTest;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KCRServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/** The one-line auto intake actions: lower/raise/stow a side, the Flower arm, and the lowerFirst runs. */
public class IntakeActionsTest {
    private final double pivotMs = IntakeConfig.PIVOT_MOVE_MS;
    private final double flowerMs = IntakeConfig.FLOWER_MOVE_MS;

    private final Servo pivotServoA = FlowerMechInterlockTest.fakeServo();
    private final Servo pivotServoB = FlowerMechInterlockTest.fakeServo();
    private final CRServo flowerServo = FlowerMechInterlockTest.fakeCRServo();
    private final DcMotor motorA = FlowerMechInterlockTest.fakeMotor();
    private final DcMotor motorB = FlowerMechInterlockTest.fakeMotor();
    private IntakeSide sideA;
    private IntakeSide sideB;
    private FlowerMech flowerMech;
    private Intake intake;

    @Before
    public void setUp() {
        IntakeConfig.PIVOT_MOVE_MS = 20;
        IntakeConfig.FLOWER_MOVE_MS = 20;
        sideA = new IntakeSide(A, motorA, new KServo(pivotServoA, 1e6, 255, 0, false));
        sideB = new IntakeSide(B, motorB, new KServo(pivotServoB, 1e6, 255, 0, false));
        flowerMech = new FlowerMech(new KCRServo(flowerServo, false), sideA);
        intake = new Intake(sideA, sideB, flowerMech);
    }

    @After
    public void restoreConfig() {
        IntakeConfig.PIVOT_MOVE_MS = pivotMs;
        IntakeConfig.FLOWER_MOVE_MS = flowerMs;
    }

    @Test
    public void sideLower_lowersAndWaitsPivotMove() throws Exception {
        long start = System.currentTimeMillis();
        runToDone(new IntakeSideLower(sideA));
        assertTrue(System.currentTimeMillis() - start >= IntakeConfig.PIVOT_MOVE_MS);
        assertEquals(ModuleConfig.INTAKE_PIVOT_A_DOWN_POS, pivotServoA.getPosition(), 1e-9);
        assertTrue(sideA.isDown());
    }

    @Test
    public void sideRaise_raisesFlowerFirstThenSide() throws Exception {
        runToDone(new IntakeSideLower(sideA));
        assertTrue(flowerMech.lower());
        runToDone(new IntakeSideRaise(sideA));
        assertEquals(IntakeConfig.FLOWER_UP_POWER, flowerServo.getPower(), 1e-9);
        assertEquals(ModuleConfig.INTAKE_PIVOT_A_UP_POS, pivotServoA.getPosition(), 1e-9);
        assertTrue(sideA.isUp());
    }

    @Test
    public void stow_stopsAndRaisesEverything() throws Exception {
        runToDone(new IntakeSideLower(sideA));
        runToDone(new IntakeSideLower(sideB));
        assertTrue(flowerMech.lower());
        intake.run(1);
        runToDone(new IntakeStow(intake));
        assertEquals(0, motorA.getPower(), 1e-9);
        assertEquals(0, motorB.getPower(), 1e-9);
        assertEquals(IntakeConfig.FLOWER_UP_POWER, flowerServo.getPower(), 1e-9);
        assertEquals(ModuleConfig.INTAKE_PIVOT_A_UP_POS, pivotServoA.getPosition(), 1e-9);
        assertEquals(ModuleConfig.INTAKE_PIVOT_B_UP_POS, pivotServoB.getPosition(), 1e-9);
    }

    @Test
    public void flowerLower_timesOutWhileSideIsUp() throws Exception {
        long start = System.currentTimeMillis();
        runToDone(new FlowerLower(flowerMech, 40));
        assertTrue(System.currentTimeMillis() - start >= 40);
        assertEquals(IntakeConfig.FLOWER_UP_POWER, flowerServo.getPower(), 1e-9);
    }

    @Test
    public void flowerLower_thenRaise_onceSideIsDown() throws Exception {
        runToDone(new IntakeSideLower(sideA));
        runToDone(new FlowerLower(flowerMech, 1000));
        assertTrue(flowerMech.isDown());
        runToDone(new FlowerRaise(flowerMech));
        assertTrue(flowerMech.isUp());
    }

    @Test
    public void runIntakeTime_lowerFirst_lowersSide_threeArgFormDoesNot() throws Exception {
        runToDone(new RunIntakeTime(sideB, 10, 1));
        assertEquals(ModuleConfig.INTAKE_PIVOT_B_UP_POS, pivotServoB.getPosition(), 1e-9);
        runToDone(new RunIntakeTime(sideB, 10, 1, true));
        assertEquals(ModuleConfig.INTAKE_PIVOT_B_DOWN_POS, pivotServoB.getPosition(), 1e-9);
        assertEquals(0, motorB.getPower(), 1e-9);
    }

    @Test
    public void intakeFullAction_oneSide_lowersItsSide() throws Exception {
        Stopper stopper = new Stopper(new KServo(FlowerMechInterlockTest.fakeServo(), 1e6, 255, 0, false));
        runToDone(new IntakeFullAction(stopper, sideB, 10, 1));
        assertEquals(ModuleConfig.INTAKE_PIVOT_B_DOWN_POS, pivotServoB.getPosition(), 1e-9);
    }

    @Test
    public void driveDirectionIntakeAction_setDone_stopsAndRaisesBoth() {
        runToDone(new IntakeSideLower(sideA));
        runToDone(new IntakeSideLower(sideB));
        intake.run(1);
        DriveDirectionIntakeAction action = new DriveDirectionIntakeAction(intake);
        action.setIsDone(true);
        assertEquals(0, motorA.getPower(), 1e-9);
        assertEquals(0, motorB.getPower(), 1e-9);
        assertEquals(ModuleConfig.INTAKE_PIVOT_A_UP_POS, pivotServoA.getPosition(), 1e-9);
        assertEquals(ModuleConfig.INTAKE_PIVOT_B_UP_POS, pivotServoB.getPosition(), 1e-9);
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
