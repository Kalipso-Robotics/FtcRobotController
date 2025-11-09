package com.kalipsorobotics.actions.shooter.pusher;


import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.actions.intake.RunIntakeUntilFullSpeed;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.utilities.KLog;

public class PushBall extends KActionSet {

    public PushBall(Stopper stopper, Intake intake) {
        KServoAutoAction release = new KServoAutoAction(stopper.getStopper(), stopper.STOPPER_SERVO_OPEN_POS);
        release.setName("release");
        this.addAction(release);

//        KLog.d("teleop", "pusher Started");
//        KCRServoAutoAction pushRight = new KCRServoAutoAction(pusher.getKickerRight(), 1);
//        pushRight.setName("pushRight");
//        this.addAction(pushRight, release);
//
//        KCRServoAutoAction pushLeft = new KCRServoAutoAction(pusher.getKickerLeft(), 1);
//        pushLeft.setName("pushLeft");
//        this.addAction(pushLeft);

        RunIntakeUntilFullSpeed untilShootingDone = new RunIntakeUntilFullSpeed(intake);
        untilShootingDone.setName("untilShootingDone");
//        untilShootingDone.setDependentActions(pushLeft, pushRight);
        this.addAction(untilShootingDone);

        KLog.d("teleop", "shooting done.(based off intake)");
        WaitAction wait = new WaitAction(1000);
        wait.setDependentActions(untilShootingDone);
        this.addAction(wait);

        KServoAutoAction block = new KServoAutoAction(stopper.getStopper(), stopper.STOPPER_SERVO_CLOSED_POS);
        block.setName("block");
        this.addAction(block);
        block.setDependentActions(wait, untilShootingDone);

//        KCRServoAutoAction stopPushRight = new KCRServoAutoAction(pusher.getKickerRight(), 0);
//        stopPushRight.setName("stopPushRight");
//        stopPushRight.setDependentActions(wait, untilShootingDone);
//        this.addAction(stopPushRight);
//
//        KCRServoAutoAction stopPushLeft = new KCRServoAutoAction(pusher.getKickerLeft(), 0);
//        stopPushLeft.setName("stopPushLeft");
//        stopPushLeft.setDependentActions(wait, untilShootingDone);
//        this.addAction(stopPushLeft);

        KLog.d("teleop", "pusher Stopped(based off intake)");

    }

}
