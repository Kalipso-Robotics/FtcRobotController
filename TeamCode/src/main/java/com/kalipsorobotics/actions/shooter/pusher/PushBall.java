package com.kalipsorobotics.actions.shooter.pusher;


import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.actions.intake.RunIntakeUntilFullSpeed;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.KLog;

public class PushBall extends KActionSet {

    public PushBall(Stopper stopper, Intake intake, Shooter shooter) {
        KServoAutoAction openStopper = new KServoAutoAction(stopper.getStopper(), stopper.STOPPER_SERVO_OPEN_POS);
        openStopper.setName("openStopper");
        this.addAction(openStopper);

        KLog.d("PushAllBalls", "openStopper " + openStopper.getIsDone());

        RunIntakeUntilFullSpeed untilShootingDone = new RunIntakeUntilFullSpeed(intake);
        untilShootingDone.setName("untilShootingDone");
//        untilShootingDone.setDependentActions(pushLeft, pushRight);
        this.addAction(untilShootingDone);

        KLog.d("PushAllBalls", "run intake " + untilShootingDone.getIsDone());


        KLog.d("teleop", "pusher Stopped(based off intake)");

    }

}
