package com.kalipsorobotics.decode;

import com.kalipsorobotics.cameraVision.AllianceSetup;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class BlueTeleOpTemp extends RedFarTeleOp {

    public BlueTeleOpTemp() {
        allianceSetup = AllianceSetup.BLUE;
    }

}
