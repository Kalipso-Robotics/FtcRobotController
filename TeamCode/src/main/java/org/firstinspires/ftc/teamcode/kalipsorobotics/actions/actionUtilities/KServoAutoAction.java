package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class KServoAutoAction extends Action {

    protected KServo kServo;
    protected double targetPos;

    ElapsedTime actionTime;

    public KServoAutoAction(KServo kServo, double targetPos) {
        this.kServo = kServo;
        this.targetPos = targetPos;
        this.actionTime = new ElapsedTime();
    }

    @Override
    public void update() {
        KLog.d("servo_action", () -> "running update  " + targetPos + "currentTime  " + kServo.getTime() +
                "port number  " + kServo.getPortNumber());
        if (isDone) {
            KLog.d("servo_action", () -> "done for  " + targetPos + "currentTime  " + kServo.getTime() +
                    "port number  " + kServo.getPortNumber());
            return;
        }

        if (!hasStarted) {
            actionTime.reset();
            hasStarted = true;
        }

        KLog.d("servo_action", () -> "not done setting target positions  " + targetPos + "currentTime  " + kServo.getTime() +
                "port number  " + kServo.getPortNumber());
        kServo.setTargetPosition(targetPos);

//        if (Math.abs(kServo.getTargetPosition() - kServo.getCurrentPosition()) < 0.01) {
//            KLog.d("servo_action", "done" + targetPos + "currentPos  " + kServo.getCurrentPosition() +
//                    "port number  " + kServo.getPortNumber());
//            isDone = true;
//        }

        isDone = kServo.isDone();
        if (isDone) {
            KLog.d("servo_action", () -> "done" + targetPos + "current time  " + kServo.getTime() +
                    "port number  " + kServo.getPortNumber());
            KLog.d("ActionTime", () -> this.getName() + " done in " + actionTime + " ms");
        }

    }

}
