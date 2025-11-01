package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.KCRServoAutoAction;
import com.kalipsorobotics.modules.Pusher;

public class PushBall extends KActionSet {

    public PushBall(Pusher pusher) {

        KCRServoAutoAction kickRight = new KCRServoAutoAction(pusher.getKickerRight(), 1);
        kickRight.setName("kickRight");
        this.addAction(kickRight);

        KCRServoAutoAction kickLeft = new KCRServoAutoAction(pusher.getKickerLeft(), 1);
        kickLeft.setName("kickLeft");
        this.addAction(kickLeft);

    }

}
