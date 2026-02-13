package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig.intakePower;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;

public class RunIntake extends Action {
    Intake intake;

    public RunIntake(Intake intake) {
        this.intake = intake;
    }

    @Override
    protected void update() {
        intake.getIntakeMotor().setPower(intakePower);
        isDone = true;
    }
}