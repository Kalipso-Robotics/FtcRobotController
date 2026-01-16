package com.kalipsorobotics.actions.autoActions.pathActions;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.actions.drivetrain.ActivateBraking;
import com.kalipsorobotics.actions.drivetrain.ReleaseBrakeAction;
import com.kalipsorobotics.actions.intake.IntakeFullAction;
import com.kalipsorobotics.actions.shooter.ShooterReady;
import com.kalipsorobotics.actions.shooter.ShooterRun;
import com.kalipsorobotics.actions.shooter.ShooterStop;
import com.kalipsorobotics.actions.shooter.stopper.CloseStopperAction;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.DriveBrake;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.navigation.SpeedAdaptivePurePursuitAction;
import com.kalipsorobotics.utilities.OpModeUtilities;


public class RampCycleAction extends KActionSet {
    RoundTripAction trip2 = null;
    PurePursuitAction moveToBall;
    OpModeUtilities opModeUtilities;
    DriveTrain driveTrain;
    TurretAutoAlign turretAutoAlign;
    Shooter shooter;
    Stopper stopper;
    Intake intake;
    Point targetPoint;
    Point launchPos;
    double waitForShooterReadyMS;

    public RampCycleAction(OpModeUtilities opModeUtilities, DriveTrain driveTrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake,
                           Point targetPoint, Point launchPos, double waitForShooterReadyMS) {
        this.opModeUtilities = opModeUtilities;
        this.driveTrain = driveTrain;
        this.turretAutoAlign = turretAutoAlign;
        this.shooter = shooter;
        this.stopper = stopper;
        this.intake = intake;
        this.targetPoint = targetPoint;
        this.launchPos = launchPos;
        this.waitForShooterReadyMS = waitForShooterReadyMS;

        moveToBall = new PurePursuitAction(driveTrain);
        moveToBall.setName("rampCycleTrip1");
        this.addAction(moveToBall);

        trip2 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake,targetPoint, launchPos, 500);
        trip2.setName("rampCycleTrip2");
        trip2.getMoveToBall().clearPoints();
        trip2.getMoveToBall().setFinalSearchRadius(200);
        trip2.getMoveToBall().setMaxTimeOutMS(4000);
        this.addAction(trip2);
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

        ShooterRun shooterRun = new ShooterRun(opModeUtilities, shooter, targetPoint, launchPos);
        shooterRun.setName("shooterRun");  // FIX: Correct name (was "shooterReady")
        shooterRun.setDependentActions();
        this.addAction(shooterRun);

        IntakeFullAction intakeFullAction = new IntakeFullAction(stopper, intake, 5000);
        intakeFullAction.setName("intake");
        this.addAction(intakeFullAction);

        WaitAction waitAction = new WaitAction(1000);
        waitAction.setName("waitForIntake");
        waitAction.setDependentActions(moveToBall);
        this.addAction(waitAction);

        trip2.setDependentActions(waitAction);

        ShooterStop shooterStop = new ShooterStop(shooterRun);
        shooterStop.setName("shooterStop");
        shooterStop.setDependentActions(trip2);
        this.addAction(shooterStop);
    }

    public PurePursuitAction getMoveToBall() {
        return moveToBall;
    }

}
