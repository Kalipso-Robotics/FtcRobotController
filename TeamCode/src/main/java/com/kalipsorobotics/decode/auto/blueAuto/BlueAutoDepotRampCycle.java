package com.kalipsorobotics.decode.auto.blueAuto;

import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.decode.auto.redAuto.RedAutoDepotRampCycle;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueAutoDepotRampCycle")
public class BlueAutoDepotRampCycle extends RedAutoDepotRampCycle {
    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.BLUE;
        SharedData.setAllianceColor(this.allianceColor);
    }

}