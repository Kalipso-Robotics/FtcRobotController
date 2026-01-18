package com.kalipsorobotics.actions.autoActions.pathActions;

import static com.kalipsorobotics.actions.intake.IntakeConfig.intakeFromRampTime;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.actions.intake.IntakeConfig;
import com.kalipsorobotics.actions.intake.IntakeFullAction;
import com.kalipsorobotics.actions.shooter.ShooterRun;
import com.kalipsorobotics.actions.shooter.ShooterStop;
import com.kalipsorobotics.actions.shooter.stopper.CloseStopperAction;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.utilities.OpModeUtilities;


public class RampCycleAction extends KActionSet {
    RoundTripAction tripToShoot = null;
    PurePursuitAction moveToRamp;
    OpModeUtilities opModeUtilities;
    IntakeFullAction intakeFullAction;
    DriveTrain driveTrain;
    TurretAutoAlign turretAutoAlign;
    Shooter shooter;
    Stopper stopper;
    Intake intake;
    Point targetPoint;
    Point launchPos;
    double waitForShooterReadyMS;
    double waitTime;
    ShooterRun shooterRun;

    public RampCycleAction(OpModeUtilities opModeUtilities, DriveTrain driveTrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake,
                           Point targetPoint, Point launchPos, double waitForShooterReadyMS, double waitTime) {
        this.opModeUtilities = opModeUtilities;
        this.driveTrain = driveTrain;
        this.turretAutoAlign = turretAutoAlign;
        this.shooter = shooter;
        this.stopper = stopper;
        this.intake = intake;
        this.targetPoint = targetPoint;
        this.launchPos = launchPos;
        this.waitForShooterReadyMS = waitForShooterReadyMS;
        this.waitTime = waitTime;

        moveToRamp = new PurePursuitAction(driveTrain);
        moveToRamp.setName("rampCycleTrip1");
        moveToRamp.setPathAngleTolerance(10);
        moveToRamp.setFinalSearchRadius(100);
        moveToRamp.setFinalAngleLockingThresholdDegree(10);
        moveToRamp.setMaxTimeOutMS(3000);
        this.addAction(moveToRamp);

        tripToShoot = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake,targetPoint, launchPos, 500);
        tripToShoot.setName("rampCycleTrip2");
        tripToShoot.getMoveToBall().clearPoints();
        tripToShoot.getMoveToBall().setFinalSearchRadius(200);
        tripToShoot.getMoveToBall().setMaxTimeOutMS(4000);
        tripToShoot.getMoveToBall().setFinalAngleLockingThresholdDegree(45);
        tripToShoot.getMoveToBall().setPathAngleTolerance(50);
        this.addAction(tripToShoot);
    }

    public RampCycleAction(OpModeUtilities opModeUtilities, DriveTrain driveTrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake,
                           Point targetPoint, Point launchPos, double waitForShooterReadyMS) {
        this(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, targetPoint, launchPos, waitForShooterReadyMS, 1000);

    }

    @Override
    public void initActions(){

        WaitAction waitUntilShootRun = new WaitAction(waitForShooterReadyMS);
        waitUntilShootRun.setName("waitUntilShootReady");
        this.addAction(waitUntilShootRun);

        //x=1405.56 (55.34 in), y=-1201.27 (-47.29 in), theta=-0.8324 (-4
        CloseStopperAction closeStopperAction = new CloseStopperAction(stopper);
        closeStopperAction.setName("closeStopper");
        this.addAction(closeStopperAction);

        shooterRun = new ShooterRun(opModeUtilities, shooter, targetPoint, launchPos);
        shooterRun.setName("shooterRun");
        shooterRun.setDependentActions();
        this.addAction(shooterRun);

        intakeFullAction = new IntakeFullAction(stopper, intake, intakeFromRampTime, 1);
        intakeFullAction.setName("intake");
        intakeFullAction.setDependentActions(moveToRamp);
        this.addAction(intakeFullAction);

        WaitAction waitAction = new WaitAction(waitTime);
        waitAction.setName("waitForIntake");
        waitAction.setDependentActions(moveToRamp);
        this.addAction(waitAction);
        // how does this wait do anything? -darren

        tripToShoot.setDependentActions(waitAction);

        ShooterStop shooterStop = new ShooterStop(shooterRun);
        shooterStop.setName("shooterStop");
        shooterStop.setDependentActions(tripToShoot);
        this.addAction(shooterStop);
    }

    public PurePursuitAction getMoveToRamp() {
        return moveToRamp;
    }

    public RoundTripAction getTripToShoot() {
        return tripToShoot;
    }

    @Override
    protected void beforeUpdate() {
        super.beforeUpdate();

        KLog.d("RampCycle", String.format("[%s] Status - MoveToRamp: %s, Intake: %s, ShooterRun: %s, TripToShoot: %s",
                getName() != null ? getName() : "unnamed",
                moveToRamp.getIsDone() ? "DONE" : "NOT DONE",
                intakeFullAction.getIsDone() ? "DONE" : "NOT DONE",
                shooterRun.getIsDone() ? "DONE" : "NOT DONE",
                tripToShoot.getIsDone() ? "DONE" : "NOT DONE"));
        KLog.d("RampCycle", String.format("[%s] MoveToRamp isWithinRange=%b",
                getName() != null ? getName() : "unnamed",
                moveToRamp.isWithinRange()));
    }

    @Override
    public void afterUpdate() {
        if (moveToRamp.getIsDone()) {
            intakeFullAction.setIsDone(true);
            KLog.d("RampCycle", String.format("[%s] IntakeFullAction stopped by MoveToRamp completion",
                    getName() != null ? getName() : "unnamed"));
        }
    }

}
