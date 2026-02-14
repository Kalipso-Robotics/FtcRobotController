package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;

public class IntakeStop extends Action {

    Intake intake;

    public IntakeStop(Intake intake) {
        this.intake = intake;
    }


    @Override
    public void update() {
        if (isDone) {
            return;
        }
        intake.getIntakeMotor().setPower(0);
        isDone = true;
    }
}
