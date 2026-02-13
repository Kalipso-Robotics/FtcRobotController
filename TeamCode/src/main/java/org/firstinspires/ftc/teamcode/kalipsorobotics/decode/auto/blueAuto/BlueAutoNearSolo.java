package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.blueAuto;

import org.firstinspires.ftc.teamcode.kalipsorobotics.cameraVision.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto.RedAutoNear;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto.RedAutoNearSolo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueAutoNearSolo extends RedAutoNearSolo {
    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.BLUE;
        SharedData.setAllianceColor(this.allianceColor);
        TurretConfig.TICKS_INIT_OFFSET = (int) Math.round(-((TurretConfig.TICKS_PER_ROTATION * TurretConfig.BIG_TO_SMALL_PULLEY) / 2)) * allianceColor.getPolarity(); //offset by 180 deg
    }
}
