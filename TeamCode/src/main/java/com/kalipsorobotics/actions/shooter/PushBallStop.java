package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.KCRServoAutoAction;
import com.kalipsorobotics.modules.Pusher;

public class PushBallStop extends KActionSet {

    public PushBallStop(Pusher pusher) {

        KCRServoAutoAction kickRight = new KCRServoAutoAction(pusher.getKickerRight(), 0);
        kickRight.setName("kickRight");
        this.addAction(kickRight);

        KCRServoAutoAction kickLeft = new KCRServoAutoAction(pusher.getKickerLeft(), 0);
        kickLeft.setName("kickLeft");
        this.addAction(kickLeft);

    }

}
