package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.drivetrain.ActivateBraking;
import com.kalipsorobotics.actions.drivetrain.ReleaseBraking;
import com.kalipsorobotics.actions.shooter.pusher.PushBall;
import com.kalipsorobotics.actions.turret.TurretAutoAlignTeleop;
import com.kalipsorobotics.actions.turret.TurretReadyLimelight;
import com.kalipsorobotics.actions.turret.TurretStop;
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
    private TurretAutoAlignTeleop turretAutoAlignTeleop;
    private double targetRPS;
    private double targetHoodPos;

    private boolean hasTurnedLimelightOn = false;

    private ShooterReady shooterReady;

    private PushBall pushBall;

    private TurretReadyLimelight turretReadyLimelight;

    public ShootAllAction(Stopper stopper, Intake intake, Shooter shooter, DriveBrake driveBrake, ShooterRun shooterRun, TurretAutoAlignTeleop turretAutoAlignTeleop, double targetRPS, double targetHoodPos) {
        this.stopper = stopper;
        this.intake = intake;
        this.shooter = shooter;
        this.driveBrake = driveBrake;
        this.shooterRun = shooterRun;
        this.turretAutoAlignTeleop = turretAutoAlignTeleop;
        this.targetRPS = targetRPS;
        this.targetHoodPos = targetHoodPos;

        shooterRun.setShooterRunMode(ShooterRunMode.SHOOT_USING_TARGET_RPS_HOOD);
        shooterRun.setTargetRPS(targetRPS);
        shooterRun.setTargetHoodPosition(targetHoodPos);

        generateBasicAction(shooterRun, stopper, intake, shooter, turretAutoAlignTeleop);
    }

    public ShootAllAction(Stopper stopper, Intake intake, Shooter shooter, DriveBrake driveBrake, ShooterRun shooterRun, TurretAutoAlignTeleop turretAutoAlignTeleop) {
        this.stopper = stopper;
        this.intake = intake;
        this.shooter = shooter;
        this.driveBrake = driveBrake;
        this.shooterRun = shooterRun;
        this.turretAutoAlignTeleop = turretAutoAlignTeleop;

        shooterRun.setShooterRunMode(ShooterRunMode.SHOOT_USING_CURRENT_POINT);

        generateBasicAction(shooterRun, stopper, intake, shooter, turretAutoAlignTeleop);
    }

    public ShooterRun getShooterRun() {
        return shooterRun;
    }

    public ShooterReady getShooterReady() {
        return shooterReady;
    }


    private void generateBasicAction(ShooterRun shooterRun, Stopper stopper, Intake intake, Shooter shooter, TurretAutoAlignTeleop turretAutoAlignTeleop) {

        ActivateBraking activateBraking = new ActivateBraking(driveBrake);
        activateBraking.setName("ActivateBraking");
        this.addAction(activateBraking);

        shooterReady = new ShooterReady(shooterRun);
        shooterReady.setName("ready");
        this.addAction(shooterReady);

        turretReadyLimelight = new TurretReadyLimelight(turretAutoAlignTeleop);
        turretReadyLimelight.setName("turretReady");
        this.addAction(turretReadyLimelight);

        pushBall = new PushBall(stopper, intake, shooter);
        pushBall.setName("pushAllBalls");
        pushBall.setDependentActions(shooterReady, turretReadyLimelight);
        pushBall.getRunUntilFullSpeed().setFullSpeedDurationMs(500);
        this.addAction(pushBall);

        TurretStop turretStop = new TurretStop(turretAutoAlignTeleop);
        turretStop.setName("turretStop");
        turretStop.setDependentActions(pushBall);
        this.addAction(turretStop);

        ReleaseBraking releaseBraking = new ReleaseBraking(driveBrake);
        releaseBraking.setName("ReleaseBraking");
        releaseBraking.setDependentActions(pushBall);
        this.addAction(releaseBraking);

    }

    @Override
    public void beforeUpdate() {
        if (isDone) {
            return;
        }
        if (!hasStarted) {
            turretAutoAlignTeleop.runWithOdometryAndLimelight();
            hasStarted = true;
        }
    }

    public void setTurretReady(boolean isDone) {
        turretReadyLimelight.setIsDone(isDone);
    }
}
