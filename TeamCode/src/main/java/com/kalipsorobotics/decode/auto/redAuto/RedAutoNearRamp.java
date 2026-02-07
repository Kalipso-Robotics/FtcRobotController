package com.kalipsorobotics.decode.auto.redAuto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedAutoNearRamp extends RedAutoNearRampThirdSpike {

    @Override
    protected void handleTrip5() {
        trip5Spike = null;
        trip5Ramp = generateRampTrip();
        trip5Ramp.setName("trip5Ramp");
        trip5Ramp.setDependentActions(trip4);
        redAutoNear.addAction(trip5Ramp);

    }

}
