package com.kalipsorobotics.decode;

import com.kalipsorobotics.actions.autoActions.pathActions.RoundTripAction;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.shooter.Shooter;

public class RedAutoNearSecondSpike extends RedAutoNear{
    Point nearLaunchPoint =  new Point(SHOOT_NEAR_X, SHOOT_NEAR_Y);

    @Override
    public void handleTrip3() {
//============================ Trip 3 (Intake Second Spike) ============================
        trip3 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), nearLaunchPoint.multiplyY(allianceColor.getPolarity()), 0);
        trip3.setName("trip2");
        trip3.getMoveToBall().addPoint(1375, 600 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip3.getMoveToBall().addPoint(1375, 1025 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        // move to lever
        trip3.getMoveToBall().addPoint(1500, 1110 * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());

//        trip2.getMoveToBall().addPoint(1500, 900 * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
//        trip2.getMoveToBall().addPoint(1500, 1050 * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
        // move to launch
        trip3.getMoveToBall().addPoint(nearLaunchPoint.getX(), nearLaunchPoint.getY() * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
        trip3.setDependentActions(trip2);
        trip3.getMoveToBall().setWithinRangeRadiusMM(150);
        trip3.getMoveToBall().setMaxTimeOutMS(9000);
        trip3.getMoveToBall().setFinalSearchRadius(200);
        redAutoNear.addAction(trip3);
    }

    @Override
    public void handleTrip4() {
        trip4 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), nearLaunchPoint.multiplyY(allianceColor.getPolarity()), 0);
        trip4.setName("trip2");
        trip4.getMoveToBall().addPoint(1375, 600 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip4.getMoveToBall().addPoint(1375, 1025 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        // move to lever
        trip4.getMoveToBall().addPoint(1500, 1110 * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());

//        trip2.getMoveToBall().addPoint(1500, 900 * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
//        trip2.getMoveToBall().addPoint(1500, 1050 * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
        // move to launch
        trip4.getMoveToBall().addPoint(nearLaunchPoint.getX(), nearLaunchPoint.getY() * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
        trip4.setDependentActions(trip3);
        trip4.getMoveToBall().setWithinRangeRadiusMM(150);
        trip4.getMoveToBall().setMaxTimeOutMS(9000);
        trip4.getMoveToBall().setFinalSearchRadius(200);
        redAutoNear.addAction(trip4);
    }







}
