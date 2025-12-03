package com.kalipsorobotics.actions.autoActions.pathActions;

import static com.kalipsorobotics.decode.RedAutoDepot.SHOOT_FAR_X;
import static com.kalipsorobotics.decode.RedAutoDepot.SHOOT_FAR_Y;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.actions.turret.TurretConfig;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.OpModeUtilities;

/*
175 1145 111
65 1145 109
265 1185 170
25 1200 170
130 780 90
SHOOT_FAR_X SHOOT_FAR_Y
 */


public class DepotRoundTrip extends KActionSet {

    RoundTripAction trip;
    public DepotRoundTrip(OpModeUtilities opModeUtilities, DriveTrain drivetrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake, Point target, Point launchPos, double waitForShooterReadyMS, AllianceColor allianceColor) {
        trip = new RoundTripAction(opModeUtilities, drivetrain, turretAutoAlign, shooter, stopper, intake, target, launchPos, 2000);
        trip.setName("trip");
        trip.getMoveToBall().setLookAheadRadius(75);
        trip.getMoveToBall().setMaxTimeOutMS(8000);
        trip.getMoveToBall().setFinalSearchRadius(100);
        trip.getMoveToBall().addPoint(15, 600 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip.getMoveToBall().addPoint(15, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip.getMoveToBall().addPoint(15,  1168 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip.getMoveToBall().addPoint(223, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip.getMoveToBall().addPoint(223,  1168 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip.getMoveToBall().addPoint(SHOOT_FAR_X, SHOOT_FAR_Y * allianceColor.getPolarity() , 90 * allianceColor.getPolarity());
        this.addAction(trip);

        turretAutoAlign.setToleranceDeg(0.5);
    }

    public RoundTripAction getTrip() {
        return trip;
    }


}
