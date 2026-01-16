package com.kalipsorobotics.decode.auto.blueAuto;

import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.decode.TeleOpV2;
import com.kalipsorobotics.decode.auto.redAuto.RedAutoNear;
import com.kalipsorobotics.decode.auto.redAuto.RedAutoNearV2;
import com.kalipsorobotics.decode.configs.TurretConfig;
import com.kalipsorobotics.decode.configs.V2ConfigHelper;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueAutoNearV2")
public class BlueAutoNearV2 extends RedAutoNearV2 {
    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.BLUE;
        SharedData.setAllianceColor(this.allianceColor);
        TurretConfig.TICKS_INIT_OFFSET = (int) Math.round(-((TurretConfig.TICKS_PER_ROTATION * TurretConfig.BIG_TO_SMALL_PULLEY) / 2)) * allianceColor.getPolarity(); //offset by 180 deg

        V2ConfigHelper.configRobotV2();
    }

}