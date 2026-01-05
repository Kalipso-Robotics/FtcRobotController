package com.kalipsorobotics.decode;

import com.kalipsorobotics.actions.turret.TurretConfig;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueAutoNearSecondSpike extends RedAutoNearSecondSpike{

    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.BLUE;
        SharedData.setAllianceColor(this.allianceColor);
        TurretConfig.TICKS_INIT_OFFSET = (int) Math.round(-((Turret.TICKS_PER_ROTATION * Turret.BIG_TO_SMALL_PULLEY) / 2)) * allianceColor.getPolarity(); //offset by 180 deg
    }

}
