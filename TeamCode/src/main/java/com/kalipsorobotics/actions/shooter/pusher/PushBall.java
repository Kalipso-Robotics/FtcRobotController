package com.kalipsorobotics.actions.shooter.pusher;


import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.actions.intake.IntakeConfig;
import com.kalipsorobotics.actions.intake.RunIntakeTime;
import com.kalipsorobotics.decode.configs.ModuleConfig;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.SharedData;

public class PushBall extends KActionSet {
//    private final RunIntakeUntilFullSpeed runUntilFullSpeed;
    public PushBall(Stopper stopper, Intake intake) {

        KLog.d("PushAllBalls", "current pos " + SharedData.getOdometryWheelIMUPosition());

        KServoAutoAction openStopper = new KServoAutoAction(stopper.getStopper(), ModuleConfig.STOPPER_SERVO_OPEN_POS);
        openStopper.setName("openStopper");
        this.addAction(openStopper);

        KLog.d("PushAllBalls", "openStopper " + openStopper.getIsDone());

        RunIntakeTime runIntakeTime = new RunIntakeTime(intake, IntakeConfig.shootTimeMS, IntakeConfig.intakePower);
        runIntakeTime.setName("runIntakeTime");
        runIntakeTime.setDependentActions(openStopper);
        this.addAction(runIntakeTime);
//
//        runUntilFullSpeed = new RunIntakeUntilFullSpeed(intake);
//        runUntilFullSpeed.setDependentActions(openStopper);
//        runUntilFullSpeed.setName("untilShootingDone");
////        untilShootingDone.setDependentActions(pushLeft, pushRight);
//        this.addAction(runUntilFullSpeed);
//
//        KLog.d("PushAllBalls", "run intake " + runUntilFullSpeed.getIsDone());


        KLog.d("teleop", "pusher Stopped(based off intake)");

    }

//    public RunIntakeUntilFullSpeed getRunUntilFullSpeed() {
//        return runUntilFullSpeed;
//    }

}
