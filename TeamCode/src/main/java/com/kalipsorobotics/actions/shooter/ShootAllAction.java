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
        this(stopper, intake, shooter, target, 0, 0);
    }

    public ShootAllAction(Stopper stopper, Intake intake, Shooter shooter, double targetRPS, double targetHoodPos) {
        this(stopper, intake, shooter, null, targetRPS, targetHoodPos);
    }

    private ShootAllAction(Stopper stopper, Intake intake, Shooter shooter, Point target, double targetRPS, double targetHoodPos) {
        ShooterRun shooterRun = new ShooterRun(shooter, target, LaunchPosition.AUTO, targetRPS, targetHoodPos, 0);
        shooterRun.setName("ShooterReady");
        this.addAction(shooterRun);

        ShooterRun shooterMaintain = new ShooterRun(shooter, target, LaunchPosition.AUTO, targetRPS, targetHoodPos, ShooterConfig.maintainTimeOutMS);
        shooterMaintain.setName("ShooterMaintain");
        this.addAction(shooterMaintain);
        shooterMaintain.setDependentActions(shooterRun);

        PushBall pushAllBalls = new PushBall(stopper, intake, shooter);
        pushAllBalls.setName("pushAllBalls");
        pushAllBalls.setDependentActions(shooterRun);
        this.addAction(pushAllBalls);

        ShooterStop shooterStop = new ShooterStop(shooterRun);
        shooterStop.setName("shooterStop");
        shooterStop.setDependentActions(pushAllBalls, shooterMaintain);
        this.addAction(shooterStop);

    }
}
