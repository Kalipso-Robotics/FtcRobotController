package com.kalipsorobotics.modules;

import com.kalipsorobotics.utilities.KCRServo;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.CRServo;

public class Pusher {

    private final OpModeUtilities opModeUtilities;

    private KCRServo kickerRight;
    private KCRServo kickerLeft;

    public Pusher(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        CRServo kickerRightHW = opModeUtilities.getHardwareMap().crservo.get("pusherRight");
        CRServo kickerLeftHW = opModeUtilities.getHardwareMap().crservo.get("pusherLeft");
        this.kickerLeft = new KCRServo(kickerLeftHW,false);
        this.kickerRight = new KCRServo(kickerRightHW,true);
    }

    public KCRServo getKickerLeft() {
        return kickerLeft;
    }

    public KCRServo getKickerRight() {
        return kickerRight;
    }

}
