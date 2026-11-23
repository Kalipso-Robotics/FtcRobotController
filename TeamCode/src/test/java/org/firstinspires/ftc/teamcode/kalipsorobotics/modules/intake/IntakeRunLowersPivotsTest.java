package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake;

import static org.junit.Assert.assertEquals;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.RunIntake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.RunIntakeTime;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KCRServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.junit.Before;
import org.junit.Test;

/** Running the whole intake lowers both pivots, so the rollers never spin in the air. */
public class IntakeRunLowersPivotsTest {

    private Servo pivotA;
    private Servo pivotB;
    private Intake intake;

    @Before
    public void setUp() {
        pivotA = FlowerMechInterlockTest.fakeServo();
        pivotB = FlowerMechInterlockTest.fakeServo();
        IntakeSide sideA = new IntakeSide(BeamBreakPosition.IntakeSide.A, FlowerMechInterlockTest.fakeMotor(), new KServo(pivotA));
        IntakeSide sideB = new IntakeSide(BeamBreakPosition.IntakeSide.B, FlowerMechInterlockTest.fakeMotor(), new KServo(pivotB));
        intake = new Intake(sideA, sideB, new FlowerMech(new KCRServo(FlowerMechInterlockTest.fakeCRServo(), false), sideA));
    }

    private void assertBothDown() {
        assertEquals(ModuleConfig.INTAKE_PIVOT_A_DOWN_POS, pivotA.getPosition(), 1e-9);
        assertEquals(ModuleConfig.INTAKE_PIVOT_B_DOWN_POS, pivotB.getPosition(), 1e-9);
    }

    @Test
    public void runIntake_lowersBothSides() {
        new RunIntake(intake).updateCheckDone();
        assertBothDown();
    }

    @Test
    public void runIntakeTime_wholeIntake_lowersBothSides() {
        new RunIntakeTime(intake, 1000, 1).updateCheckDone();
        assertBothDown();
    }
}
