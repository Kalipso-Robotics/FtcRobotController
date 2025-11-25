package com.kalipsorobotics.decode;

import com.kalipsorobotics.cameraVision.AllianceColor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueAutoNearZone")
public class BlueAutoNear extends RedAutoNear {

    public BlueAutoNear() {
        this.allianceColor = AllianceColor.BLUE;
    }

}