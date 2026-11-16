package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.blueAuto;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig.shouldBangBangCompensate;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto.RedAutoDepot;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto.RedAutoDepotNoSpike;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

@Disabled
@Autonomous
public class BlueAutoDepotNoSpike extends RedAutoDepotNoSpike {
    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.BLUE;
        SharedData.setAllianceColor(allianceColor);
        TurretConfig.TRIM_DEG = 0;
    }
}
