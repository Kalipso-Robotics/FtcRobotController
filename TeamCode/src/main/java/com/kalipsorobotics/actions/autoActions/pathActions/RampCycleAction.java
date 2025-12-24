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

    public RampCycleAction(OpModeUtilities opModeUtilities, DriveTrain driveTrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake, AllianceColor allianceColor, Point shooterPoint) {
        autoActionPath(driveTrain, allianceColor, stopper, intake, opModeUtilities, turretAutoAlign, shooter, shooterPoint, 180);
    }

    public RampCycleAction(OpModeUtilities opModeUtilities, DriveTrain driveTrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake, AllianceColor allianceColor, Point shooterPoint, double shooterHeading) {
        autoActionPath(driveTrain, allianceColor, stopper, intake, opModeUtilities, turretAutoAlign, shooter, shooterPoint, shooterHeading);
    }

    private void autoActionPath(DriveTrain driveTrain, AllianceColor allianceColor, Stopper stopper, Intake intake, OpModeUtilities opModeUtilities, TurretAutoAlign turretAutoAlign, Shooter shooter, Point shooterPoint, double shooterHeading) {
        PurePursuitAction trip1 = new PurePursuitAction(driveTrain);
        trip1.setName("rampCycleTrip1");
        trip1.addPoint(1151, 842 * allianceColor.getPolarity(), 47 * allianceColor.getPolarity());
        trip1.addPoint(1201, 1040 * allianceColor.getPolarity(), 50 * allianceColor.getPolarity());
        this.addAction(trip1);

        //x=1405.56 (55.34 in), y=-1201.27 (-47.29 in), theta=-0.8324 (-4
        CloseStopperAction closeStopperAction = new CloseStopperAction(stopper);
        closeStopperAction.setName("closeStopper");
        this.addAction(closeStopperAction);

        ShooterRun shooterRun = new ShooterRun(shooter, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), shooterPoint);
        shooterRun.setName("shooterRun");  // FIX: Correct name (was "shooterReady")
        this.addAction(shooterRun);

        IntakeFullAction intakeFullAction = new IntakeFullAction(stopper, intake, 5000);
        intakeFullAction.setName("intake");
        this.addAction(intakeFullAction);

        WaitAction waitAction = new WaitAction(1000);
        waitAction.setName("waitForIntake");
        waitAction.setDependentActions(trip1);
        this.addAction(waitAction);

        trip2 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), shooterPoint.multiplyY(allianceColor.getPolarity()), 500);
        trip2.setName("rampCycleTrip2");
        trip2.getMoveToBall().clearPoints();
        trip2.getMoveToBall().setFinalSearchRadius(200);
        trip2.getMoveToBall().setMaxTimeOutMS(4000);
        trip2.getMoveToBall().addPoint(shooterPoint.getX(), shooterPoint.getY() * allianceColor.getPolarity(), shooterHeading * allianceColor.getPolarity());
        trip2.setDependentActions(waitAction);
        this.addAction(trip2);

        ShooterStop shooterStop = new ShooterStop(shooterRun);
        shooterStop.setName("shooterStop");
        shooterStop.setDependentActions(trip2);
        this.addAction(shooterStop);
    }
}
