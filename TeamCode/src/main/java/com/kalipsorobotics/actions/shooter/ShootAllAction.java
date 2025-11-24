package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.shooter.pusher.PushBall;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.Shooter;


public class ShootAllAction extends KActionSet {

    ShooterRun shooterRun;

    public ShootAllAction(Stopper stopper, Intake intake, Shooter shooter, double targetRPS, double targetHoodPos) {
        shooterRun = new ShooterRun(shooter, targetRPS, targetHoodPos);
        shooterRun.setName("ShooterReady");
        this.addAction(shooterRun);

        generateBasicAction(shooterRun, stopper, intake, shooter);
    }

    public ShootAllAction(Stopper stopper, Intake intake, Shooter shooter, Point targetPoint) {
        shooterRun = new ShooterRun(shooter, targetPoint);
        shooterRun.setName("ShooterReady");
        this.addAction(shooterRun);

        generateBasicAction(shooterRun, stopper, intake, shooter);
    }

    public ShooterRun getShooterRun() {
        return shooterRun;
    }

    private void generateBasicAction(ShooterRun shooterRun, Stopper stopper, Intake intake, Shooter shooter) {
        ShooterReady ready = new ShooterReady(shooterRun);
        ready.setName("ready");
        this.addAction(ready);

        PushBall pushAllBalls = new PushBall(stopper, intake, shooter);
        pushAllBalls.setName("pushAllBalls");
        pushAllBalls.setDependentActions(ready);
        this.addAction(pushAllBalls);

        ShooterStop shooterStop = new ShooterStop(shooterRun);
        shooterStop.setName("shooterStop");
        shooterStop.setDependentActions(pushAllBalls, ready);
        this.addAction(shooterStop);
    }
}
