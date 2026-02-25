package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions.DepotRoundTrip;

@Autonomous
public class RedAutoDepotScanning extends RedAutoDepot {

    @Override
    protected void handleTrip3() {
        trip3 = generateRetryTrip(trip2, DepotTrips.SWEEP_400);
        autoDepot.addAction(trip3);
    }

    @Override
    protected void handleTrip4() {
        trip4 = generateRetryTrip(trip3, DepotTrips.SWEEP_800);
        trip4.getTrip().getMoveToBall().setMaxTimeOutMS(4000);
        autoDepot.addAction(trip4);
    }

    @Override
    protected void handleTrip5() {
        trip5 = generateRetryTrip(trip4, DepotTrips.SWEEP_0);
        trip4.getTrip().getMoveToBall().setMaxTimeOutMS(4000);
        autoDepot.addAction(trip5);
    }

    @Override
    protected DepotRoundTrip handleTrip6(DepotRoundTrip trip5) {
        DepotRoundTrip trip6 = generateRetryTrip(trip5, DepotTrips.SWEEP_TUNNEL);
        trip6.getTrip().getMoveToBall().setMaxTimeOutMS(5000);
        return trip6;
    }

}
