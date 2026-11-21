package org.firstinspires.ftc.teamcode.kalipsorobotics.decode;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;

/** The same TeleOp for the BLUE alliance. */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = " TeleOp Blue")
public class TeleOpBlue extends TeleOp {
    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.BLUE;
        SharedData.setAllianceColor(allianceColor);
    }
}
