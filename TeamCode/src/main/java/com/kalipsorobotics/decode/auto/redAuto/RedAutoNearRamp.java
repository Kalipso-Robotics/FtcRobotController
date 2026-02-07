package com.kalipsorobotics.decode.auto.redAuto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedAutoNearRamp extends RedAutoNearRampThirdSpike {

    @Override
    protected void handleTrip4() {
        trip4Spike = null;
        trip4Ramp = generateRampTrip();
        trip4Ramp.setName("trip4Ramp");
        trip4Ramp.setDependentActions(trip3);
        redAutoNear.addAction(trip4Ramp);

    }

}
