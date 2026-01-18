package com.kalipsorobotics.decode.auto.redAuto;

import com.kalipsorobotics.actions.autoActions.pathActions.RampCycleAction;
import com.kalipsorobotics.actions.autoActions.pathActions.RoundTripAction;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedAutoNearSecondSpike extends RedAutoNear {
    Point nearLaunchPoint =  new Point(SHOOT_NEAR_X, SHOOT_NEAR_Y);

    @Override
    public void handleTrip3() {
        trip3 = new RampCycleAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), nearLaunchPoint.multiplyY(allianceColor.getPolarity()), 0, 500);

        trip3.setName("trip3");
        trip3.getTripToShoot().getMoveToBall().clearPoints();
        trip3.getMoveToRamp().clearPoints();
        //lever
        trip3.getMoveToRamp().addPoint(1700, 1040 * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());

        //tunnel
        trip3.getTripToShoot().getMoveToBall().addPoint(1700, 900 * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
        trip3.getTripToShoot().getMoveToBall().addPoint(1350, 1075 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());

        // move to launch
        trip3.getTripToShoot().getMoveToBall().addPoint(thirdShootPoint.getX(), (thirdShootPoint.getY()) * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
        trip3.setDependentActions(trip2);

        trip3.getTripToShoot().getMoveToBall().setLookAheadRadius(100);
        trip3.getTripToShoot().getMoveToBall().setWithinRangeRadiusMM(100);
        redAutoNear.addAction(trip3);
    }

    //============================ Trip 4 (Intake Second Spike) ============================
    @Override
    public void handleTrip4() {
        trip4 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), nearLaunchPoint.multiplyY(allianceColor.getPolarity()), 0);
        trip4.setName("trip4");

        //no lever on last trip

        //tunnel
        trip4.getMoveToBall().addPoint(1700, 800 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip4.getMoveToBall().addPoint(1350, 1075 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());

        // move to launch
        trip4.getMoveToBall().addPoint(FINAL_SHOOT_NEAR_X, FINAL_SHOOT_NEAR_Y * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
        trip4.getMoveToBall().setFinalAngleLockingThresholdDegree(45);
        trip4.setDependentActions(trip3);
        trip4.getMoveToBall().setLookAheadRadius(100);
        trip4.getMoveToBall().setWithinRangeRadiusMM(100);
        trip4.getMoveToBall().setMaxTimeOutMS(9000);
        trip4.getMoveToBall().setFinalSearchRadius(100);
        redAutoNear.addAction(trip4);
    }







}
