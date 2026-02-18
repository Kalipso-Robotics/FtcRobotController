package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions.DepotRoundTrip;

@Autonomous
public class RedAutoDepotNoSpike extends RedAutoDepot {

    @Override
    protected DepotRoundTrip handleTrip6(DepotRoundTrip trip5) {
        return generateRetryTrip(trip5, DepotTrips.SWEEP_OUT);
    }

}
