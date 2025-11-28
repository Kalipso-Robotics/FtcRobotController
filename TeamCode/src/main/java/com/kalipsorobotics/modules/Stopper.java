package com.kalipsorobotics.modules;

import com.kalipsorobotics.utilities.KCRServo;
import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.Servo;

public class Stopper {

    public static final double STOPPER_SERVO_CLOSED_POS = 0.55;
    public static final double STOPPER_SERVO_OPEN_POS = 0.7;

    private final OpModeUtilities opModeUtilities;

    private KCRServo kickerRight;
    private KCRServo kickerLeft;

    private KServo stopper;

    public Stopper(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
//        CRServo kickerRightHW = opModeUtilities.getHardwareMap().crservo.get("pusherRight");
//        CRServo kickerLeftHW = opModeUtilities.getHardwareMap().crservo.get("pusherLeft");
        Servo stopper = opModeUtilities.getHardwareMap().servo.get("stopper");
//        this.kickerLeft = new KCRServo(kickerLeftHW,false);
//        this.kickerRight = new KCRServo(kickerRightHW,true);
        this.stopper = new KServo(stopper,KServo.AXON_MAX_SPEED, 255, 0, false);
        this.stopper.setPosition(STOPPER_SERVO_CLOSED_POS);
    }

    public KCRServo getKickerLeft() {
        return kickerLeft;
    }

    public KCRServo getKickerRight() {
        return kickerRight;
    }

    public KServo getStopper() {return stopper;}

    public void setPosition(double target) {
        stopper.setPosition(target);
    }

}
