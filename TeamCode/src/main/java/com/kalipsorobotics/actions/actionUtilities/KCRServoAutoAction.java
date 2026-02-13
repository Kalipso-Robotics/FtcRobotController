package com.kalipsorobotics.actions.actionUtilities;

import com.kalipsorobotics.utilities.KCRServo;

public class KCRServoAutoAction extends Action {

    protected KCRServo kCRServo;

    protected double power;

    public KCRServoAutoAction(KCRServo kCRServo, double power) {
        this.kCRServo = kCRServo;
        this.power = power;
    }

    @Override
    public void update() {
        if (isDone) {
            return;
        }

        kCRServo.setPower(power);
        isDone = true;
    }
}
