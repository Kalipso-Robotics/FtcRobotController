package com.kalipsorobotics.actions.autoActions.intakeActions;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.modules.Intake;

public class IntakeRun extends Action {

    Intake intake;

    public IntakeRun(Intake intake) {
        this.intake = intake;
    }

    @Override
    protected boolean checkDoneCondition() {
        return isDone;
    }

    @Override
    protected void update() {
        intake.getIntakeMotor().setPower(1);
    }
}
