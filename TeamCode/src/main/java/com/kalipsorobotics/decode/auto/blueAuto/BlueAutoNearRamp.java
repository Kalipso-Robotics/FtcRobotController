package com.kalipsorobotics.decode.auto.blueAuto;

import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.decode.auto.redAuto.RedAutoNearRamp;
import com.kalipsorobotics.decode.configs.TurretConfig;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous
public class BlueAutoNearRamp extends RedAutoNearRamp {

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.BLUE;
        SharedData.setAllianceColor(this.allianceColor);
        TurretConfig.TICKS_INIT_OFFSET = (int) Math.round(-((TurretConfig.TICKS_PER_ROTATION * TurretConfig.BIG_TO_SMALL_PULLEY) / 2)) * allianceColor.getPolarity(); //offset by 180 deg
    }

}
