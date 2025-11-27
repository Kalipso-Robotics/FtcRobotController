package com.kalipsorobotics.decode;

import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueAutoNearZone")
public class BlueAutoNear extends RedAutoNear {
    @Override
    protected void initializeAllianceColor() {
        this.allianceColor = AllianceColor.BLUE;
        SharedData.setAllianceColor(this.allianceColor);
    }

}