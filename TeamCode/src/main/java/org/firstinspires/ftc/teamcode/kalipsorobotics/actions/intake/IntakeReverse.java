package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;

public class IntakeReverse extends Action {

    Intake intake;

    public IntakeReverse(Intake intake) {
        this.intake = intake;
    }

    @Override
    protected void update() {
        intake.getIntakeMotor().setPower(-1);
        isDone = true;
    }
}
