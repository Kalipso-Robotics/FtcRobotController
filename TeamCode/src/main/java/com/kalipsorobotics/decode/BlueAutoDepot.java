package com.kalipsorobotics.decode;

import com.kalipsorobotics.cameraVision.AllianceSetup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueAutoDepot")
public class BlueAutoDepot extends RedAutoDepot {
    public BlueAutoDepot() {
        allianceSetup = AllianceSetup.BLUE;
    }
}
