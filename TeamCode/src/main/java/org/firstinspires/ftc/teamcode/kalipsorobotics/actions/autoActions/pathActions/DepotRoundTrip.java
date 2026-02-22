package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto.RedAutoDepot.SHOOT_FAR_X;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto.RedAutoDepot.SHOOT_FAR_Y;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.cameraVision.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

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
        trip = new RoundTripAction(opModeUtilities, drivetrain, turretAutoAlign, shooter, stopper, intake, target, launchPos, 2000, true, true);
        trip.setName("trip");
        trip.getMoveToBall().setLookAheadRadius(150);
        trip.getMoveToBall().setMaxTimeOutMS(4000);
        trip.getMoveToBall().setFinalSearchRadius(150);
        trip.getMoveToBall().setFinalAngleLockingThresholdDegree(30);
        trip.getPurePursuitReadyShooting().setDistanceThresholdMM(100);
        trip.getPurePursuitReadyIntakeStop().setDistanceThresholdMM(400);
        trip.getMoveToBall().setPathAngleTolerance(10);
//        trip.getPushBall().getRunUntilFullSpeed().setFullSpeedDurationMs(200);
        trip.setShouldShooterStop(false);
        //trip.getMoveToBall().addPoint(15, 600 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        //trip.getMoveToBall().addPoint(15, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        //trip.getMoveToBall().addPoint(15,  1168 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip.getMoveToBall().addPoint(100, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip.getMoveToBall().addPoint(100, 1140 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip.getMoveToBall().addPoint(SHOOT_FAR_X, SHOOT_FAR_Y * allianceColor.getPolarity() , 90 * allianceColor.getPolarity());

        trip.getIntakeFullAction().getRunIntakeTime().setTimeMS(2600);

        this.addAction(trip);

        turretAutoAlign.setToleranceDeg(0.5);
    }

    public RoundTripAction getTrip() {
        return trip;
    }


}
