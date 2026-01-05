package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.drivetrain.ActivateBraking;
import com.kalipsorobotics.actions.drivetrain.ReleaseBraking;
import com.kalipsorobotics.actions.shooter.pusher.PushBall;
import com.kalipsorobotics.actions.turret.TurretAutoAlignTeleOp;
import com.kalipsorobotics.actions.turret.TurretReadyLimelight;
import com.kalipsorobotics.actions.turret.TurretStop;
import com.kalipsorobotics.modules.DriveBrake;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.modules.shooter.ShooterRunMode;
import com.kalipsorobotics.test.turret.TurretRunMode;
import com.kalipsorobotics.utilities.KLog;


public class ShootAllAction extends KActionSet {

    private Stopper stopper;
    private Intake intake;
    private Shooter shooter;
    private DriveBrake driveBrake;

    private ShooterRun shooterRun;
    private TurretAutoAlignTeleOp turretAutoAlignTeleop;
    private double targetRPS;
    private double targetHoodPos;

    private boolean hasTurnedLimelightOn = false;

    private ShooterReady shooterReady;

    private PushBall pushBall;

    private TurretReadyLimelight turretReadyLimelight;

    public ShootAllAction(Stopper stopper, Intake intake, Shooter shooter, DriveBrake driveBrake, ShooterRun shooterRun, TurretAutoAlignTeleOp turretAutoAlignTeleop, double targetRPS, double targetHoodPos) {
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

    public ShootAllAction(Stopper stopper, Intake intake, Shooter shooter, DriveBrake driveBrake, ShooterRun shooterRun, TurretAutoAlignTeleOp turretAutoAlignTeleop) {
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



    private void generateBasicAction(ShooterRun shooterRun, Stopper stopper, Intake intake, Shooter shooter, TurretAutoAlignTeleOp turretAutoAlignTeleop) {

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
        pushBall.getRunUntilFullSpeed().setFullSpeedDurationMs(1000);
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
            //Refresh targetRPS
            turretAutoAlignTeleop.setTurretRunMode(TurretRunMode.RUN_WHILE_SHOOTING);
            shooterRun.update();
            hasStarted = true;
        }
    }

    @Override
    public void afterUpdate() {
        if (pushBall.getIsDone()) {
            double maintainRPSValue = shooterRun.getTargetRPS() * 0.9;
            shooterRun.setTargetRPS(maintainRPSValue);
            shooterRun.setShooterRunMode(ShooterRunMode.SHOOT_USING_TARGET_RPS_HOOD);
            KLog.d("ShooterRun", "Maintaining " + maintainRPSValue + " RPS after Running ShootAllAction");
        }
    }

    public void setTurretReady(boolean isDone) {
        turretReadyLimelight.setIsDone(isDone);
    }
}
