package com.kalipsorobotics.decode.auto.redAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "RedAutoDepotSpikeMark")
public class RedAutoDepotSpikeMark extends RedAutoDepot {

    @Override
    public void addPointsToTrip1SpikeMark() {
        trip1.getTrip().getMoveToBall().clearPoints();

        trip1.getTrip().getMoveToBall().addPoint(727, 110 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getTrip().getMoveToBall().addPoint(727, 1150 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getTrip().getMoveToBall().addPoint(SHOOT_FAR_X, SHOOT_FAR_Y * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
    }
}
