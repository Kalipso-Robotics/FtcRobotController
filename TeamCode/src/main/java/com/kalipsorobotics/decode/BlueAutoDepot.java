package com.kalipsorobotics.decode;

import com.kalipsorobotics.cameraVision.AllianceColor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueAutoDepot")
public class BlueAutoDepot extends RedAutoDepot {
    public BlueAutoDepot() {
        this.allianceColor = AllianceColor.BLUE;
    }
}
