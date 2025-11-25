package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;

public class IntakeRunFullSpeed extends Action {
    Intake intake;

    public IntakeRunFullSpeed(Intake intake) {
        this.intake = intake;
    }

    @Override
    protected void update() {
        intake.getIntakeMotor().setPower(1);
        isDone = true;
    }
}
