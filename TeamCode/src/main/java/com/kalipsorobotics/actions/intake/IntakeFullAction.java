package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.decode.configs.ModuleConfig;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;

public class IntakeFullAction extends KActionSet {
    private RunIntakeTime runIntakeTime;

    public IntakeFullAction(Stopper stopper, Intake intake, double timeMS, double power){

        KServoAutoAction closeStopper = new KServoAutoAction(stopper.getStopper(), ModuleConfig.STOPPER_SERVO_CLOSED_POS);
        closeStopper.setName("closeStopper");
        this.addAction(closeStopper);

        runIntakeTime = new RunIntakeTime(intake, timeMS, power);
        runIntakeTime.setName("intakeRun");
        this.addAction(runIntakeTime);

//        stopIntake = new IntakeStop(intake);
//        stopIntake.setName("stopIntake");
//        stopIntake.setDependentActions(intakeAll);
//        this.addAction(stopIntake);

    }

    public RunIntakeTime getRunIntakeTime() {
        return runIntakeTime;
    }

    //    public void stop() {
//        if (isDone) {
//            return;
//        }
//        stopIntake.update();
//        this.isDone = true;
//    }
}


