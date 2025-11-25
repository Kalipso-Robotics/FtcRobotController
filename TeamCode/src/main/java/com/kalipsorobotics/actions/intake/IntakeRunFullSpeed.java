package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;

public class IntakeRunFullSpeed extends Action {
    Intake intake;
    Stopper stopper;

    public IntakeRunFullSpeed(Intake intake, Stopper stopper) {
        this.intake = intake;
        this.stopper = stopper;
    }

    @Override
    protected void update() {
        intake.getIntakeMotor().setPower(1);
        stopper.getStopper().setPosition(stopper.STOPPER_SERVO_CLOSED_POS);
        isDone = true;
    }
}
