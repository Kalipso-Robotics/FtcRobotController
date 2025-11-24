package com.kalipsorobotics.decode;

import com.kalipsorobotics.cameraVision.AllianceSetup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Blue Teleop")
public class BlueFarTeleop extends RedFarTeleOp{
    public BlueFarTeleop() {
        allianceSetup = AllianceSetup.BLUE;
    }
}
