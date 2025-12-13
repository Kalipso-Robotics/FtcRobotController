package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.drivetrain.ActivateBraking;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.drivetrain.ReleaseBraking;
import com.kalipsorobotics.actions.shooter.pusher.PushBall;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.actions.turret.TurretAutoAlignLimelight;
import com.kalipsorobotics.actions.turret.TurretReadyLimelight;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.DriveBrake;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.test.turret.TurretReady;
import com.kalipsorobotics.utilities.KLog;


public class ShootAllAction extends KActionSet {

    public ShooterRun shooterRun;
    TurretReadyLimelight turretReadyLimelight;
    ShooterReady ready;
    PushBall pushAllBalls;
    ShooterStop shooterStop;
    TurretAutoAlign turretAutoAlign;

    public ShootAllAction(Stopper stopper, Intake intake, Shooter shooter, TurretAutoAlignLimelight turretAutoAlignLimelight, double targetRPS, double targetHoodPos, DriveBrake driveBrake, ActivateBraking activateBraking, ReleaseBraking releaseBraking) {

        activateBraking = new ActivateBraking(driveBrake);
        releaseBraking = new ReleaseBraking(driveBrake);
        shooterRun = new ShooterRun(shooter, targetRPS, targetHoodPos);
        shooterRun.setName("ShooterReady");
        this.addAction(shooterRun);
        generateBasicAction(shooterRun, stopper, intake, shooter, turretAutoAlignLimelight, activateBraking, releaseBraking);
    }

    public ShootAllAction(Stopper stopper, Intake intake, Shooter shooter, TurretAutoAlignLimelight turretAutoAlignLimelight, Point targetPoint, DriveBrake driveBrake, ActivateBraking activateBraking, ReleaseBraking releaseBraking) {

        activateBraking = new ActivateBraking(driveBrake);
        releaseBraking = new ReleaseBraking(driveBrake);
        shooterRun = new ShooterRun(shooter, targetPoint);
        shooterRun.setName("ShooterReady");
        this.addAction(shooterRun);

        generateBasicAction(shooterRun, stopper, intake, shooter, turretAutoAlignLimelight, activateBraking, releaseBraking);
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

    private void generateBasicAction(ShooterRun shooterRun, Stopper stopper, Intake intake, Shooter shooter, TurretAutoAlignLimelight turretAutoAlignLimelight, ActivateBraking activateBraking, ReleaseBraking releaseBraking) {

        activateBraking.setName("ActivateBraking");
        this.addAction(activateBraking);

        ready = new ShooterReady(shooterRun);
        ready.setName("ready");
        this.addAction(ready);

        turretReadyLimelight = new TurretReadyLimelight(turretAutoAlignLimelight);
        turretReadyLimelight.setName("turretReady");
        this.addAction(turretReadyLimelight);

        pushAllBalls = new PushBall(stopper, intake, shooter);
        pushAllBalls.setName("pushAllBalls");
        pushAllBalls.setDependentActions(ready, turretReadyLimelight);
        this.addAction(pushAllBalls);

        shooterStop = new ShooterStop(shooterRun);
        shooterStop.setName("shooterStop");
        shooterStop.setDependentActions(pushAllBalls, ready);
        this.addAction(shooterStop);

        releaseBraking.setName("ReleaseBraking");
        releaseBraking.setDependentActions(shooterStop);
        this.addAction(releaseBraking);
    }

    public void setTurretReady(boolean isDone) {
        turretReadyLimelight.setIsDone(isDone);
    }
}
