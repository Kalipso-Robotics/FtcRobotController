package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;

public class IntakeFullAction extends KActionSet {
    private final RunIntakeTime runIntakeTime;

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


