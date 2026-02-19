package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;

public class IntakeStop extends Action {

    Intake intake;

    IntakeFullAction intakeFullAction;

    public IntakeStop(Intake intake) {
        this.intake = intake;
    }

    public IntakeStop(IntakeFullAction intakeFullAction) {
        this.intakeFullAction = intakeFullAction;
    }


    @Override
    public void update() {
        if (isDone) {
            return;
        }
        if (intake != null) {
            intake.getIntakeMotor().setPower(0);
        }
        if (intakeFullAction != null) {
            intakeFullAction.stopAndSetDone();
        }

        isDone = true;


    }
}
