package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.modules.shooter.Shooter;

public class KickBall extends KActionSet {

    public static double KICK_RIGHT_POS = 0.53; //up to go down
    public static double KICK_LEFT_POS = 0.74; //down to go down
    public static double STATIC_RIGHT_POS = 0.86;
    public static double STATIC_LEFT_POS = 0.4;

    public KickBall(Shooter shooter) {

        KServoAutoAction kickRight = new KServoAutoAction(shooter.getKickerRight(), KICK_RIGHT_POS);
        kickRight.setName("kickRight");
        this.addAction(kickRight);
        KServoAutoAction kickLeft = new KServoAutoAction(shooter.getKickerLeft(), KICK_LEFT_POS);
        kickLeft.setName("kickLeft");
        this.addAction(kickLeft);
        WaitAction waitAction = new WaitAction(100);
        waitAction.setName("waitAction");
        this.addAction(waitAction);
        KServoAutoAction staticRight = new KServoAutoAction(shooter.getKickerRight(), STATIC_RIGHT_POS);
        staticRight.setName("staticRight");
        this.addAction(staticRight);
        staticRight.setDependentActions(kickRight, waitAction);
        KServoAutoAction staticLeft = new KServoAutoAction(shooter.getKickerLeft(), STATIC_LEFT_POS);
        staticLeft.setName("staticLeft");
        this.addAction(staticLeft);
        staticLeft.setDependentActions(kickLeft, waitAction);
    }

}
