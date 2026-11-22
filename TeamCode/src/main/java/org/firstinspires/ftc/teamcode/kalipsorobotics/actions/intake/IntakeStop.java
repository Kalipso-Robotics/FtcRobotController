package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

public class IntakeStop extends Action {
    public static final String TAG = "Intaking_IntakeStop";

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
<<<<<<< HEAD
            intake.getIntakeMotorFlower().setPower(0);
=======
            intake.stop();
>>>>>>> 9c27d2d1b179e33b4e0870dc56c7d8a16e21afc9
        }
        if (intakeFullAction != null) {
            intakeFullAction.stopAndSetDone();
        }

        isDone = true;
        KLog.d(TAG, () -> "name=" + getName() + " event=stop");
    }
}
