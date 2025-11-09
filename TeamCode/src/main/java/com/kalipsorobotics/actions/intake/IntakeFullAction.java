package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.RunUntilStallAction;
import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.revolverActions.RevolverIntakeAction;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.modules.TripleColorSensor;

public class IntakeFullAction extends KActionSet {
    public IntakeFullAction(Intake intake, double maxTimeout){

        IntakeRun intakeRun = new IntakeRun(intake);
        intakeRun.setName("intakeRun");
        this.addAction(intakeRun);

        RunUntilStallAction intakeAll = new RunUntilStallAction(intake.getIntakeMotor(), 1, maxTimeout);
        intakeAll.setName("intakeAll");
        intakeAll.setDependentActions(intakeRun);
        this.addAction(intakeAll);

        IntakeStop intakeStop = new IntakeStop(intake);
        intakeStop.setName("intakeStop");
        intakeStop.setDependentActions(intakeAll);
        this.addAction(intakeStop);

    }
}
