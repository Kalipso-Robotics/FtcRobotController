package com.kalipsorobotics.decode;

import com.kalipsorobotics.cameraVision.AllianceSetup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueAutoNearZone")
public class BlueAutoNear extends RedAutoNear {

    public BlueAutoNear() {
        allianceSetup = AllianceSetup.BLUE;
    }

}