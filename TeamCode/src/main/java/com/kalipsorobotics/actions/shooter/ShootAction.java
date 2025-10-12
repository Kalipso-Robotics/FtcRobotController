package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.shooter.LaunchPosition;
import com.kalipsorobotics.modules.shooter.Shooter;

public class ShootAction extends KActionSet {

    public ShootAction(Shooter shooter, Point target, LaunchPosition launchPosition) {
        ShooterReady shooterReady = new ShooterReady(shooter, target, launchPosition);
        shooterReady.setName("ShooterReady");
        this.addAction(shooterReady);

        KickBall kickBall = new KickBall(shooter);
        kickBall.setName("KickBall");
        this.addAction(kickBall);
        kickBall.setDependentActions(shooterReady);
    }

}
