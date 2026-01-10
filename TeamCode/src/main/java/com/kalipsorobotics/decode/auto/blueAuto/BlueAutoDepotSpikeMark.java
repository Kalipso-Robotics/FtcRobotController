package com.kalipsorobotics.decode.auto.blueAuto;

import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.decode.auto.redAuto.RedAutoDepotSpikeMark;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "BlueAutoDepotSpikeMark")
public class BlueAutoDepotSpikeMark extends RedAutoDepotSpikeMark {
    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.BLUE;
        SharedData.setAllianceColor(allianceColor);
    }
}
