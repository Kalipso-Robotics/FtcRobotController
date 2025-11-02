package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.KCRServoAutoAction;
import com.kalipsorobotics.actions.intake.RunIntakeUntilFullSpeed;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Pusher;
import com.kalipsorobotics.utilities.KLog;

public class PushBall extends KActionSet {

    public PushBall(Pusher pusher, Intake intake) {
        KLog.d("teleop", "pusher Started");
        KCRServoAutoAction pushRight = new KCRServoAutoAction(pusher.getKickerRight(), 1);
        pushRight.setName("pushRight");
        this.addAction(pushRight);

        KCRServoAutoAction pushLeft = new KCRServoAutoAction(pusher.getKickerLeft(), 1);
        pushLeft.setName("pushLeft");
        this.addAction(pushLeft);

        RunIntakeUntilFullSpeed untilShootingDone = new RunIntakeUntilFullSpeed(intake);
        untilShootingDone.setName("untilShootingDone");
//        untilShootingDone.setDependentActions(pushLeft, pushRight);
        this.addAction(untilShootingDone);

        KLog.d("teleop", "shooting done.(based off intake)");

        KCRServoAutoAction stopPushRight = new KCRServoAutoAction(pusher.getKickerRight(), 0);
        stopPushRight.setName("stopPushRight");
        stopPushRight.setDependentActions(untilShootingDone);
        this.addAction(stopPushRight);

        KCRServoAutoAction stopPushLeft = new KCRServoAutoAction(pusher.getKickerLeft(), 0);
        stopPushLeft.setName("stopPushLeft");
        stopPushLeft.setDependentActions(untilShootingDone);
        this.addAction(stopPushLeft);

        KLog.d("teleop", "pusher Stopped(based off intake)");

    }

}
