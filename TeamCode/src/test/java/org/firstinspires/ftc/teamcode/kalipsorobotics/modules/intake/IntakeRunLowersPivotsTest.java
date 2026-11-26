package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;

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
        IntakeModule sideA = new IntakeModule(BeamBreakPosition.IntakeSide.A, FlowerMechInterlockTest.fakeMotor(), new KServo(pivotA), IntakeConfig.SIDE_A);
        IntakeModule sideB = new IntakeModule(BeamBreakPosition.IntakeSide.B, FlowerMechInterlockTest.fakeMotor(), new KServo(pivotB), IntakeConfig.SIDE_B);
        intake = new Intake(sideA, sideB, new FlowerMech(new KCRServo(FlowerMechInterlockTest.fakeCRServo(), false), sideA));
    }

    private void assertBothDown() {
        assertEquals(IntakeConfig.SIDE_A.pivotDownPos, pivotA.getPosition(), 1e-9);
        assertEquals(IntakeConfig.SIDE_B.pivotDownPos, pivotB.getPosition(), 1e-9);
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
