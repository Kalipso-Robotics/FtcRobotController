package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.shooter.pusher.PushBall;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Pusher;
import com.kalipsorobotics.modules.shooter.LaunchPosition;
import com.kalipsorobotics.modules.shooter.Shooter;

public class ShootAction extends KActionSet {

    public ShootAction(Pusher pusher, Intake intake, Shooter shooter, Point target, LaunchPosition launchPosition) {
        ShooterReady shooterReady = new ShooterReady(shooter, target, launchPosition);
        shooterReady.setName("ShooterReady");
        this.addAction(shooterReady);

        PushBall pushBall1 = new PushBall(pusher, intake);
        pushBall1.setName("kickBall1");
        this.addAction(pushBall1);
        pushBall1.setDependentActions(shooterReady);


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
        shooterStop.setDependentActions(pushBall1);
    }

}
