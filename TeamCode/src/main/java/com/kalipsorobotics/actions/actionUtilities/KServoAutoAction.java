package com.kalipsorobotics.actions.actionUtilities;

import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.utilities.KServo;

public class KServoAutoAction extends Action {

    protected KServo kServo;
    protected double targetPos;

    public KServoAutoAction(KServo kServo, double targetPos) {
        this.kServo = kServo;
        this.targetPos = targetPos;
    }


    @Override
    public void update() {
        KLog.d("servo_action", "running update  " + targetPos + "currentTime  " + kServo.getTime() +
                "port number  " + kServo.getPortNumber());
        if (isDone) {
            KLog.d("servo_action", "done for  " + targetPos + "currentTime  " + kServo.getTime() +
                    "port number  " + kServo.getPortNumber());

            return;
        }

        KLog.d("servo_action", "not done setting target positions  " + targetPos + "currentTime  " + kServo.getTime() +
                "port number  " + kServo.getPortNumber());
        kServo.setTargetPosition(targetPos);

//        if (Math.abs(kServo.getTargetPosition() - kServo.getCurrentPosition()) < 0.01) {
//            KLog.d("servo_action", "done" + targetPos + "currentPos  " + kServo.getCurrentPosition() +
//                    "port number  " + kServo.getPortNumber());
//            isDone = true;
//        }

        isDone = kServo.isDone();
        if (isDone) {
            KLog.d("servo_action", "done" + targetPos + "current time  " + kServo.getTime() +
                    "port number  " + kServo.getPortNumber());
        }

    }

}
