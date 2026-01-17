package com.kalipsorobotics.decode.auto.blueAuto;

import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.decode.auto.redAuto.RedAutoDepot;
import com.kalipsorobotics.decode.configs.TurretConfig;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueAutoDepot")
public class BlueAutoDepot extends RedAutoDepot {
    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.BLUE;
        SharedData.setAllianceColor(allianceColor);
        TurretConfig.TICKS_INIT_OFFSET = 0;
    }
}
