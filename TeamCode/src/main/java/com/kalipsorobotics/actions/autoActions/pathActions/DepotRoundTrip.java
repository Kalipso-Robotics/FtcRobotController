package com.kalipsorobotics.actions.autoActions.pathActions;

import static com.kalipsorobotics.decode.RedAutoDepot.SHOOT_FAR_X;
import static com.kalipsorobotics.decode.RedAutoDepot.SHOOT_FAR_Y;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
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
    public DepotRoundTrip(OpModeUtilities opModeUtilities, DriveTrain drivetrain, Shooter shooter, Stopper stopper, Intake intake, Point target, Point launchPos, double waitForShooterReadyMS, AllianceColor allianceColor) {
        RoundTripAction trip = new RoundTripAction(opModeUtilities, drivetrain, shooter, stopper, intake, target, launchPos, 2000);
        trip.setName("trip");
//        trip.getMoveToBall().addPoint(175, 1145 * allianceSetup.getPolarity(), 111 * allianceSetup.getPolarity());
//        trip.getMoveToBall().addPoint(65, 1145 * allianceSetup.getPolarity() , 109 * allianceSetup.getPolarity());
//        trip.getMoveToBall().addPoint(25, 1145 * allianceSetup.getPolarity(), 109 * allianceSetup.getPolarity());
//        trip.getMoveToBall().addPoint(130, 1145 * allianceSetup.getPolarity(), 175 * allianceSetup.getPolarity());
////        trip.getMoveToBall().addPoint(265, 1185 * allianceSetup.getPolarity() , 175 * allianceSetup.getPolarity());
////        trip.getMoveToBall().addPoint(25, 1225 * allianceSetup.getPolarity() , 175 * allianceSetup.getPolarity());
//        trip.getMoveToBall().addPoint(130, 780 * allianceSetup.getPolarity() , 90 * allianceSetup.getPolarity());

        trip.getMoveToBall().addPoint(105, 600 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip.getMoveToBall().addPoint(115, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip.getMoveToBall().addPoint(130, 1200 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());

        trip.getMoveToBall().addPoint(SHOOT_FAR_X, SHOOT_FAR_Y * allianceColor.getPolarity() , 90 * allianceColor.getPolarity());
        this.addAction(trip);
    }
}
