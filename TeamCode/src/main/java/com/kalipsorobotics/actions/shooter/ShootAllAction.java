package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.shooter.pusher.PushBall;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.test.turret.TurretReady;
import com.kalipsorobotics.utilities.KLog;


public class ShootAllAction extends KActionSet {

    public ShooterRun shooterRun;
    TurretReady turretReady;
    ShooterReady ready;
    PushBall pushAllBalls;
    ShooterStop shooterStop;
    TurretAutoAlign turretAutoAlign;

    public ShootAllAction(Stopper stopper, Intake intake, Shooter shooter, TurretAutoAlign turretAutoAlign, double targetRPS, double targetHoodPos) {
        shooterRun = new ShooterRun(shooter, targetRPS, targetHoodPos);
        shooterRun.setName("ShooterReady");
        this.addAction(shooterRun);

        generateBasicAction(shooterRun, stopper, intake, shooter, turretAutoAlign);
    }

    public ShootAllAction(Stopper stopper, Intake intake, Shooter shooter, TurretAutoAlign turretAutoAlign, Point targetPoint) {
        shooterRun = new ShooterRun(shooter, targetPoint);
        shooterRun.setName("ShooterReady");
        this.addAction(shooterRun);

        generateBasicAction(shooterRun, stopper, intake, shooter, turretAutoAlign);
    }

    public ShooterRun getShooterRun() {
        return shooterRun;
    }

    public ShooterReady getShooterReady() {
        return ready;
    }

    @Override
    protected void beforeUpdate() {
        KLog.d("ShootAllAction", "is Done: " + isDone + " ready: " + ready.getIsDone() + " pushAllBalls: " + pushAllBalls.getIsDone() + " shooterStop: " + shooterStop.getIsDone());
    }

    private void generateBasicAction(ShooterRun shooterRun, Stopper stopper, Intake intake, Shooter shooter, TurretAutoAlign turretAutoAlign) {

        ready = new ShooterReady(shooterRun);
        ready.setName("ready");
        this.addAction(ready);

        turretReady = new TurretReady(turretAutoAlign);
        turretReady.setName("turretReady");
        this.addAction(turretReady);

        pushAllBalls = new PushBall(stopper, intake, shooter);
        pushAllBalls.setName("pushAllBalls");
        pushAllBalls.setDependentActions(ready, turretReady);
        this.addAction(pushAllBalls);

        shooterStop = new ShooterStop(shooterRun);
        shooterStop.setName("shooterStop");
        shooterStop.setDependentActions(pushAllBalls, ready);
        this.addAction(shooterStop);
    }

    public void setTurretReady(boolean isDone) {
        turretReady.setIsDone(isDone);
    }
}
