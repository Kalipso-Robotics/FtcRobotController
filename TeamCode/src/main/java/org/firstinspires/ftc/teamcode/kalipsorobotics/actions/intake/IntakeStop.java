package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

public class IntakeStop extends Action {
    public static final String TAG = "Intaking_IntakeStop";

    Intake intake;

    IntakeFullAction intakeFullAction;

    IntakeRun intakeRun;

    public IntakeStop(Intake intake) {
        this.intake = intake;
    }

    /** Stops an IntakeRun (so it is done and stops re-commanding RPS) and the intake with it. */
    public IntakeStop(IntakeRun intakeRun) {
        this.intakeRun = intakeRun;
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
            intake.stop();
        }
        if (intakeRun != null) {
            intakeRun.stop();
        }
        if (intakeFullAction != null) {
            intakeFullAction.stopAndSetDone();
        }

        isDone = true;
        KLog.d(TAG, () -> "name=" + getName() + " event=stop");
    }
}
