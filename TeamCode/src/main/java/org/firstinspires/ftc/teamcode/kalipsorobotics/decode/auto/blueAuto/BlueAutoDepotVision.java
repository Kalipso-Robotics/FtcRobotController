package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.blueAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.kalipsorobotics.cameraVision.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto.RedAutoDepotVision;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * Blue Near autonomous with vision-guided ball collection.
 * Mirrors RedAutoNearVision with alliance-specific configuration.
 */
@Autonomous(name = "BlueAutoDepotVision")
public class BlueAutoDepotVision extends RedAutoDepotVision {

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.BLUE;
        SharedData.setAllianceColor(this.allianceColor);
        TurretConfig.TICKS_INIT_OFFSET = (int) Math.round(
                -((TurretConfig.TICKS_PER_ROTATION * TurretConfig.BIG_TO_SMALL_PULLEY) / 2)
        ) * allianceColor.getPolarity();
    }
}
