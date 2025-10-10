package com.kalipsorobotics.actions.revolverActions;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.autoActions.shooterActions.KickBall;
import com.kalipsorobotics.actions.shooter.ShooterReady;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.KColor;

public class RevolverShootColorAction extends KActionSet {
    public RevolverShootColorAction(Revolver revolver, Shooter shooter, KColor.Color shootColor) {
        ShooterReady shooterReady = new ShooterReady(shooter, Shooter.FAR_STARTING_POS_MM);
        shooterReady.setName("shooterReady");
        this.addAction(shooterReady);

        RevolverMoveToColorAction revolverMoveToColorAction = new RevolverMoveToColorAction(revolver, shootColor);
        revolverMoveToColorAction.setName("revolverMoveToColorAction");
        this.addAction(revolverMoveToColorAction);

        KickBall kickBall = new KickBall(shooter);
        kickBall.setName("kickBall");
        kickBall.setDependentActions(revolverMoveToColorAction, shooterReady);
        this.addAction(kickBall);
    }
}
