package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.blueAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.kalipsorobotics.cameraVision.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto.RedAutoDepot;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto.RedAutoDepotNew;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

@Autonomous(name = "BlueAutoDepotNew")
public class BlueAutoDepotNew extends RedAutoDepotNew {
    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.BLUE;
        SharedData.setAllianceColor(allianceColor);
        TurretConfig.TICKS_INIT_OFFSET = 0;
    }
}
