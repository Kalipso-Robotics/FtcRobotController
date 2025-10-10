package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.modules.Intake;

public class IntakeRun extends Action {

    Intake intake;

    public IntakeRun(Intake intake) {
        this.intake = intake;
    }

    @Override
    protected void update() {
        intake.getIntakeMotor().setPower(1);
        isDone = true;
    }
}
