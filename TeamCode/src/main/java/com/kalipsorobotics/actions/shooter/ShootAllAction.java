package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.drivetrain.ActivateBraking;
import com.kalipsorobotics.actions.drivetrain.ReleaseBraking;
import com.kalipsorobotics.actions.shooter.pusher.PushBall;
import com.kalipsorobotics.actions.turret.TurretAutoAlignLimelight;
import com.kalipsorobotics.actions.turret.TurretReadyLimelight;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.DriveBrake;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.modules.shooter.ShooterRunMode;


public class ShootAllAction extends KActionSet {

    private Stopper stopper;
    private Intake intake;
    private Shooter shooter;
    private DriveBrake driveBrake;

    private ShooterRun shooterRun;
    private TurretAutoAlignLimelight turretAutoAlignLimelight;
    private double targetRPS;
    private double targetHoodPos;

    private ShooterReady shooterReady;

    private TurretReadyLimelight turretReadyLimelight;

    public ShootAllAction(Stopper stopper, Intake intake, Shooter shooter, DriveBrake driveBrake, ShooterRun shooterRun, TurretAutoAlignLimelight turretAutoAlignLimelight, double targetRPS, double targetHoodPos) {
        this.stopper = stopper;
        this.intake = intake;
        this.shooter = shooter;
        this.driveBrake = driveBrake;
        this.shooterRun = shooterRun;
        this.turretAutoAlignLimelight = turretAutoAlignLimelight;
        this.targetRPS = targetRPS;
        this.targetHoodPos = targetHoodPos;

        shooterRun.setShooterRunMode(ShooterRunMode.SHOOT_USING_TARGET_RPS_HOOD);
        shooterRun.setTargetRPS(targetRPS);
        shooterRun.setTargetHoodPosition(targetHoodPos);

        generateBasicAction(shooterRun, stopper, intake, shooter, turretAutoAlignLimelight);
    }

    public ShootAllAction(Stopper stopper, Intake intake, Shooter shooter, DriveBrake driveBrake, ShooterRun shooterRun, TurretAutoAlignLimelight turretAutoAlignLimelight) {
        this.stopper = stopper;
        this.intake = intake;
        this.shooter = shooter;
        this.driveBrake = driveBrake;
        this.shooterRun = shooterRun;
        this.turretAutoAlignLimelight = turretAutoAlignLimelight;

        shooterRun.setShooterRunMode(ShooterRunMode.SHOOT_USING_CURRENT_POINT);

        generateBasicAction(shooterRun, stopper, intake, shooter, turretAutoAlignLimelight);
    }

    public ShooterRun getShooterRun() {
        return shooterRun;
    }

    public ShooterReady getShooterReady() {
        return shooterReady;
    }


    private void generateBasicAction(ShooterRun shooterRun, Stopper stopper, Intake intake, Shooter shooter, TurretAutoAlignLimelight turretAutoAlignLimelight) {

        ActivateBraking activateBraking = new ActivateBraking(driveBrake);
        activateBraking.setName("ActivateBraking");
        this.addAction(activateBraking);

        shooterReady = new ShooterReady(shooterRun);
        shooterReady.setName("ready");
        this.addAction(shooterReady);

        turretReadyLimelight = new TurretReadyLimelight(turretAutoAlignLimelight);
        turretReadyLimelight.setName("turretReady");
        this.addAction(turretReadyLimelight);

        PushBall pushAllBalls = new PushBall(stopper, intake, shooter);
        pushAllBalls.setName("pushAllBalls");
        pushAllBalls.setDependentActions(shooterReady, turretReadyLimelight);
        this.addAction(pushAllBalls);

        ReleaseBraking releaseBraking = new ReleaseBraking(driveBrake);
        releaseBraking.setName("ReleaseBraking");
        releaseBraking.setDependentActions(pushAllBalls);
        this.addAction(releaseBraking);

    }

    public void setTurretReady(boolean isDone) {
        turretReadyLimelight.setIsDone(isDone);
    }
}
