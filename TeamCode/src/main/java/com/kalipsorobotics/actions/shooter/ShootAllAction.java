package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.shooter.pusher.PushBall;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.LaunchPosition;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.modules.shooter.ShooterConfig;


public class ShootAllAction extends KActionSet {

    public ShootAllAction(Stopper stopper, Intake intake, Shooter shooter, Point target) {
        ShooterReady shooterReady = new ShooterReady(shooter, target, LaunchPosition.AUTO);
        shooterReady.setName("ShooterReady");
        this.addAction(shooterReady);

        ShooterReady shooterMaintain = new ShooterReady(shooter, target, LaunchPosition.AUTO, ShooterConfig.maintainTimeOutMS);
        shooterMaintain.setName("ShooterMaintain");
        this.addAction(shooterMaintain);
        shooterMaintain.setDependentActions(shooterReady);


        PushBall pushAllBalls = new PushBall(stopper, intake, shooter);
        pushAllBalls.setName("pushAllBalls");
        pushAllBalls.setDependentActions(shooterReady);
        this.addAction(pushAllBalls);

        ShooterStop shooterStop = new ShooterStop(shooter);
        shooterStop.setName("shooterStop");
        shooterStop.setDependentActions(pushAllBalls, shooterMaintain);
        this.addAction(shooterStop);

    }
}
