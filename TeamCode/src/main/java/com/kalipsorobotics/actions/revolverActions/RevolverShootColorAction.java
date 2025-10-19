package com.kalipsorobotics.actions.revolverActions;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.actions.shooter.KickBall;
import com.kalipsorobotics.actions.shooter.ShooterReady;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.MotifColor;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.modules.shooter.LaunchPosition;
import com.kalipsorobotics.modules.shooter.Shooter;

public class RevolverShootColorAction extends KActionSet {
    public RevolverShootColorAction(Revolver revolver, Shooter shooter, MotifColor shootColor, DetectColorsAction detectColorsAction) {
        RevolverMoveToColorAction revolverMoveToColorAction = new RevolverMoveToColorAction(revolver, shootColor, detectColorsAction);
        revolverMoveToColorAction.setName("revolverMoveToColorAction");
        this.addAction(revolverMoveToColorAction);

        WaitAction waitAction1 = new WaitAction(300);
        waitAction1.setName("waitAction");
        this.addAction(waitAction1);
        waitAction1.setDependentActions(revolverMoveToColorAction);

        KickBall kickBall = new KickBall(shooter);
        kickBall.setName("kickBall");
        kickBall.setDependentActions(revolverMoveToColorAction, waitAction1);
        this.addAction(kickBall);
    }
}
