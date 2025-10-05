package com.kalipsorobotics.actions.autoActions.intakeActions;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.revolverActions.RevolverIntakeAction;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Revolver;

public class IntakeFullAction extends KActionSet {
    public IntakeFullAction(Intake intake, Revolver revolver){

        IntakeRun intakeRun = new IntakeRun(intake);
        intakeRun.setName("intakeRun");
        this.addAction(intakeRun);

        RevolverIntakeAction revolverIntakeAction = new RevolverIntakeAction(revolver);
        revolverIntakeAction.setName("revolverIntakeAction");
        revolverIntakeAction.setDependentActions(intakeRun);
        this.addAction(revolverIntakeAction);

        IntakeStop intakeStop = new IntakeStop(intake);
        intakeStop.setName("intakeStop");
        intakeStop.setDependentActions(revolverIntakeAction);
        this.addAction(intakeStop);

    }
}
