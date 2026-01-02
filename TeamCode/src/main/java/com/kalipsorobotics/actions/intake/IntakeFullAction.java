package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.RunUntilStallAction;
import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;

public class IntakeFullAction extends KActionSet {
    private IntakeStop stopIntake;

    public IntakeFullAction(Stopper stopper, Intake intake, double maxTimeoutMS){

        KServoAutoAction closeStopper = new KServoAutoAction(stopper.getStopper(), stopper.STOPPER_SERVO_CLOSED_POS);
        closeStopper.setName("closeStopper");
        this.addAction(closeStopper);

        IntakeRunFullSpeed intakeRunFullSpeed = new IntakeRunFullSpeed(intake);
        intakeRunFullSpeed.setName("intakeRun");
        this.addAction(intakeRunFullSpeed);

        RunUntilStallAction intakeAll = new RunUntilStallAction(intake.getIntakeMotor(), 1, maxTimeoutMS);
        intakeAll.setName("intakeAll");
        intakeAll.setDependentActions(intakeRunFullSpeed);
        this.addAction(intakeAll);

//        stopIntake = new IntakeStop(intake);
//        stopIntake.setName("stopIntake");
//        stopIntake.setDependentActions(intakeAll);
//        this.addAction(stopIntake);

    }

//    public void stop() {
//        if (isDone) {
//            return;
//        }
//        stopIntake.update();
//        this.isDone = true;
//    }
}
