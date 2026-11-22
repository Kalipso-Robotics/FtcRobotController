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
<<<<<<< HEAD
        intake.getIntakeMotorFlower().setPower(-1);
=======
        intake.run(-1);
>>>>>>> 9c27d2d1b179e33b4e0870dc56c7d8a16e21afc9
        isDone = true;
        KLog.d(TAG, () -> "name=" + getName() + " event=reverse");
    }
}
