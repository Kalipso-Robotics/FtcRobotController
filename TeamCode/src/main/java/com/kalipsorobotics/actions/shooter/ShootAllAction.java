package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.drivetrain.ActivateBraking;
import com.kalipsorobotics.actions.drivetrain.ReleaseBraking;
import com.kalipsorobotics.actions.shooter.pusher.PushBall;
import com.kalipsorobotics.actions.turret.TurretAutoAlignTeleOp;
import com.kalipsorobotics.actions.turret.TurretReady;
import com.kalipsorobotics.actions.turret.TurretStop;
import com.kalipsorobotics.localization.ResetOdometryToPosition;
import com.kalipsorobotics.modules.DriveBrake;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.modules.shooter.ShooterRunMode;
import com.kalipsorobotics.utilities.KLog;


public class ShootAllAction extends KActionSet {

    private final Turret turret;
    private final Stopper stopper;
    private final Intake intake;
    private final Shooter shooter;
    private final DriveBrake driveBrake;

    private final ShooterRun shooterRun;
    private final TurretAutoAlignTeleOp turretAutoAlignTeleop;
    private double targetRPS;
    private double targetHoodPos;

    private final boolean hasTurnedLimelightOn = false;

    private ShooterReady shooterReady;

    private PushBall pushBall;

    private TurretReady turretReady;

    public ShootAllAction(Turret turret, Stopper stopper, Intake intake, Shooter shooter, DriveBrake driveBrake, ShooterRun shooterRun, TurretAutoAlignTeleOp turretAutoAlignTeleop, double targetRPS, double targetHoodPos) {
        this.stopper = stopper;
        this.intake = intake;
        this.shooter = shooter;
        this.driveBrake = driveBrake;
        this.shooterRun = shooterRun;
        this.turretAutoAlignTeleop = turretAutoAlignTeleop;
        this.targetRPS = targetRPS;
        this.targetHoodPos = targetHoodPos;
        this.turret = turret;

        shooterRun.setShooterRunMode(ShooterRunMode.SHOOT_USING_TARGET_RPS_HOOD);
        shooterRun.setTargetRPS(targetRPS);
        shooterRun.setTargetHoodPosition(targetHoodPos);

        generateBasicAction(shooterRun, stopper, intake, turretAutoAlignTeleop);
    }

    public ShootAllAction(Turret turret, Stopper stopper, Intake intake, Shooter shooter, DriveBrake driveBrake, ShooterRun shooterRun, TurretAutoAlignTeleOp turretAutoAlignTeleop) {
        this.stopper = stopper;
        this.intake = intake;
        this.shooter = shooter;
        this.driveBrake = driveBrake;
        this.shooterRun = shooterRun;
        this.turretAutoAlignTeleop = turretAutoAlignTeleop;
        this.turret = turret;

        shooterRun.setShooterRunMode(ShooterRunMode.SHOOT_USING_CURRENT_POINT);


        generateBasicAction(shooterRun, stopper, intake, turretAutoAlignTeleop);
    }

    public ShooterRun getShooterRun() {
        return shooterRun;
    }

    public ShooterReady getShooterReady() {
        return shooterReady;
    }



    private void generateBasicAction(ShooterRun shooterRun, Stopper stopper, Intake intake, TurretAutoAlignTeleOp turretAutoAlignTeleop) {

//        ActivateBraking activateBraking = new ActivateBraking(driveBrake);
//        activateBraking.setName("ActivateBraking");
//        this.addAction(activateBraking);

        shooterReady = new ShooterReady(shooterRun);
        shooterReady.setName("shooterReady");
        this.addAction(shooterReady);

        turretReady = new TurretReady(turretAutoAlignTeleop);
        turretReady.setName("turretReady");
        this.addAction(turretReady);

        pushBall = new PushBall(stopper, intake);
        pushBall.setName("pushAllBalls");
        pushBall.setDependentActions(shooterReady, turretReady);
//        pushBall.getRunUntilFullSpeed().setFullSpeedDurationMs(1000);
        this.addAction(pushBall);

        TurretStop turretStop = new TurretStop(turretAutoAlignTeleop);
        turretStop.setName("turretStop");
        turretStop.setDependentActions(pushBall);
        this.addAction(turretStop);


        ResetOdometryToPosition resetOdometryToPosition = new ResetOdometryToPosition(turret);
        resetOdometryToPosition.setName("resetOdometryToPosition");
        resetOdometryToPosition.setDependentActions(turretStop);
        this.addAction(resetOdometryToPosition);

//        ReleaseBraking releaseBraking = new ReleaseBraking(driveBrake);
//        releaseBraking.setName("ReleaseBraking");
//        releaseBraking.setDependentActions(pushBall);
//        this.addAction(releaseBraking);

    }

    @Override
    public void beforeUpdate() {
        if (isDone) {
            return;
        }
        if (!hasStarted) {
            //Refresh targetRPS
            shooterRun.update();
            hasStarted = true;
        }
    }

    @Override
    public void afterUpdate() {
        if (pushBall.getIsDone()) {
            double maintainRPSValue = shooterRun.getTargetRPS() * 0.925;

            shooterRun.setTargetRPS(maintainRPSValue);
            shooterRun.setShooterRunMode(ShooterRunMode.SHOOT_USING_CURRENT_POINT);
            //KLog.d("ShooterRun", "Maintaining " + maintainRPSValue + " RPS after Running ShootAllAction");
        }
    }

    public void setTurretReady(boolean isDone) {
        turretReady.setIsDone(isDone);
    }
}
