package com.kalipsorobotics.decode;

import com.kalipsorobotics.cameraVision.AllianceSetup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class BlueTeleOpTemp extends RedNearTeleop {

    public BlueTeleOpTemp() {
        allianceSetup = AllianceSetup.BLUE;
    }

}
