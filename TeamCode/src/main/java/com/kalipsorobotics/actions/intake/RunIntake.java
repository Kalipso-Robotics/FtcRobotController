package com.kalipsorobotics.actions.intake;

import static com.kalipsorobotics.actions.intake.IntakeConfig.intakePower;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;

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