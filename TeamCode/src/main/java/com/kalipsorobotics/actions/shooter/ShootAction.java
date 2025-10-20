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

        KickBall kickBall1 = new KickBall(shooter);
        kickBall1.setName("kickBall1");
        this.addAction(kickBall1);
        kickBall1.setDependentActions(shooterReady);
//
//
//        KickBall kickBall2 = new KickBall(shooter);
//        kickBall2.setName("kickBall2");
//        this.addAction(kickBall2);
//        kickBall2.setDependentActions(shooterReady, kickBall);
//
//        KickBall kickBall3 = new KickBall(shooter);
//        kickBall3.setName("kickBall3");
//        this.addAction(kickBall3);
//        kickBall3.setDependentActions(shooterReady, kickBall);

        ShooterStop shooterStop = new ShooterStop(shooter);
        shooterStop.setName("shooterStop");
        this.addAction(shooterStop);
        shooterStop.setDependentActions(kickBall1);
    }

}
