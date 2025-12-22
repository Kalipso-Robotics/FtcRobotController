package com.kalipsorobotics.actions.autoActions.pathActions;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.actions.intake.IntakeFullAction;
import com.kalipsorobotics.actions.shooter.stopper.CloseStopperAction;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.utilities.OpModeUtilities;


public class RampCycleAction extends KActionSet {

    RoundTripAction trip3 = null;

    public RampCycleAction(OpModeUtilities opModeUtilities, DriveTrain driveTrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake, AllianceColor allianceColor, Point shooterPoint) {
        trip3 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), new Point(2860, 135 * allianceColor.getPolarity()), 500);
        autoActionPath(driveTrain, allianceColor, stopper, intake, opModeUtilities, turretAutoAlign, shooter, shooterPoint);
    }
    private void autoActionPath(DriveTrain driveTrain, AllianceColor allianceColor, Stopper stopper, Intake intake, OpModeUtilities opModeUtilities, TurretAutoAlign turretAutoAlign, Shooter shooter, Point shooterPoint) {
        PurePursuitAction trip2_1 = new PurePursuitAction(driveTrain);
        trip2_1.setName("trip2.5");
        trip2_1.setFinalSearchRadius(40);
        trip2_1.addPoint(1161.85, 982.76 * allianceColor.getPolarity(), 47 * allianceColor.getPolarity());
        trip2_1.addPoint(1305, 1101 * allianceColor.getPolarity(), 55 * allianceColor.getPolarity());
        this.addAction(trip2_1);
        //x=1405.56 (55.34 in), y=-1201.27 (-47.29 in), theta=-0.8324 (-4
        CloseStopperAction closeStopperAction = new CloseStopperAction(stopper);
        closeStopperAction.setName("closeStopper");
        this.addAction(closeStopperAction);

        IntakeFullAction intakeFullAction = new IntakeFullAction(stopper, intake, 5000);
        intakeFullAction.setName("intake");
        intakeFullAction.setDependentActions(trip2_1);
        this.addAction(intakeFullAction);

        WaitAction waitAction = new WaitAction(1000);
        waitAction.setName("waitForIntake");
        waitAction.setDependentActions(trip2_1);
        this.addAction(waitAction);


        trip3 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), new Point(2860, 135 * allianceColor.getPolarity()), 500);
        trip3.setName("trip3");
        trip3.setDependentActions(waitAction);
        trip3.getMoveToBall().clearPoints();
        trip3.getMoveToBall().addPoint(shooterPoint.getX(), shooterPoint.getY() * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
        this.addAction(trip3);
    }
}
