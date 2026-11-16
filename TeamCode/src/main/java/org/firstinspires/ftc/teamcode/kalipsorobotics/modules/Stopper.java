package org.firstinspires.ftc.teamcode.kalipsorobotics.modules;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.Servo;

public class Stopper {
    public static final String TAG = "Robot_Stopper";

    private final OpModeUtilities opModeUtilities;

    private final KServo stopper;

    public Stopper(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        try {
            Servo stopper = opModeUtilities.getHardwareMap().servo.get("stopper");
            this.stopper = new KServo(stopper);
            this.stopper.setTargetPosition(ModuleConfig.STOPPER_SERVO_CLOSED_POS);
            KLog.d(TAG, "event=init stopper=closed");
        } catch (Exception e) {
            KLog.e(TAG, "event=initFailed error=" + e.getMessage());
            throw e;
        }
    }

    public KServo getStopper() {return stopper;}

    public void setPosition(double target) {
        stopper.setPosition(target);
    }

    public void open() {
        moveTo(ModuleConfig.STOPPER_SERVO_OPEN_POS, "open");
    }

    public void close() {
        moveTo(ModuleConfig.STOPPER_SERVO_CLOSED_POS, "close");
    }

    private void moveTo(double position, String event) {
        if (stopper.getTargetPosition() != position) {
            KLog.d(TAG, () -> "event=" + event);
        }
        stopper.setTargetPosition(position);
    }

}
