package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.blueAuto;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig.shouldBangBangCompensate;

import org.firstinspires.ftc.teamcode.kalipsorobotics.cameraVision.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto.RedAutoDepot;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueAutoDepot")
public class BlueAutoDepot extends RedAutoDepot {
    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.BLUE;
        SharedData.setAllianceColor(allianceColor);
        TurretConfig.TICKS_INIT_OFFSET = 0;
        shouldBangBangCompensate = true;
    }
}
