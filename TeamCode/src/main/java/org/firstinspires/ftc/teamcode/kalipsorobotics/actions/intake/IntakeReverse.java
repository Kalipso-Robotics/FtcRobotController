package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

public class IntakeReverse extends Action {
    public static final String TAG = "Intaking_IntakeReverse";

    Intake intake;

    public IntakeReverse(Intake intake) {
        this.intake = intake;
    }

    @Override
    protected void update() {
        intake.run(-1);
        isDone = true;
        KLog.d(TAG, () -> "name=" + getName() + " event=reverse");
    }
}
