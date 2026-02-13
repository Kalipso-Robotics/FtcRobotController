package org.firstinspires.ftc.teamcode.kalipsorobotics.modules;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.Servo;

public class Stopper {

    private final OpModeUtilities opModeUtilities;

    private final KServo stopper;

    public Stopper(OpModeUtilities opModeUtilities) {
        KLog.d("Stopper", "Constructor called - starting initialization");
        this.opModeUtilities = opModeUtilities;
//        CRServo kickerRightHW = opModeUtilities.getHardwareMap().crservo.get("pusherRight");
//        CRServo kickerLeftHW = opModeUtilities.getHardwareMap().crservo.get("pusherLeft");
        try {
            Servo stopper = opModeUtilities.getHardwareMap().servo.get("stopper");
            KLog.d("Stopper", "Hardware servo retrieved: " + (stopper != null ? "SUCCESS" : "NULL"));
//        this.kickerLeft = new KCRServo(kickerLeftHW,false);
//        this.kickerRight = new KCRServo(kickerRightHW,true);
            this.stopper = new KServo(stopper);
            this.stopper.setTargetPosition(ModuleConfig.STOPPER_SERVO_CLOSED_POS);
            KLog.d("Stopper", "Constructor completed successfully");
        } catch (Exception e) {
            KLog.e("Stopper", "ERROR in constructor: " + e.getMessage());
            throw e;
        }
    }

    public KServo getStopper() {return stopper;}

    public void setPosition(double target) {
        stopper.setPosition(target);
    }

}
