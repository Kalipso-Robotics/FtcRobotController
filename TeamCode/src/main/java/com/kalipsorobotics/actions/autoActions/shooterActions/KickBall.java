package com.kalipsorobotics.actions.autoActions.shooterActions;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.actions.shooter.ShooterStop;
import com.kalipsorobotics.modules.shooter.Shooter;

public class KickBall extends KActionSet {

    public KickBall(Shooter shooter) {

        KServoAutoAction kickRight = new KServoAutoAction(shooter.getKickerRight(), 0.45);
        kickRight.setName("kickRight");
        this.addAction(kickRight);
        KServoAutoAction kickLeft = new KServoAutoAction(shooter.getKickerLeft(), 0.68);
        kickLeft.setName("kickLeft");
        this.addAction(kickLeft);
        WaitAction waitAction = new WaitAction(500);
        waitAction.setName("waitAction");
        this.addAction(waitAction);
        KServoAutoAction staticRight = new KServoAutoAction(shooter.getKickerRight(), 0.85);
        staticRight.setName("staticRight");
        this.addAction(staticRight);
        staticRight.setDependentActions(kickRight, waitAction);
        KServoAutoAction staticLeft = new KServoAutoAction(shooter.getKickerLeft(), 0.43);
        staticLeft.setName("staticLeft");
        this.addAction(staticLeft);
        staticLeft.setDependentActions(kickLeft, waitAction);
        ShooterStop shooterStop = new ShooterStop(shooter);
        shooterStop.setName("shooterStop");
        this.addAction(shooterStop);
        shooterStop.setDependentActions(staticRight, staticLeft);
    }

}
