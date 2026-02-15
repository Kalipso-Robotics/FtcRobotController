package org.firstinspires.ftc.teamcode.kalipsorobotics.modules;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.Servo;

public class Tilter {

    private final OpModeUtilities opModeUtilities;

    private final KServo tilter;

    public Tilter(OpModeUtilities opModeUtilities) {
        KLog.d("Tilter", "Constructor called - starting initialization");
        this.opModeUtilities = opModeUtilities;
        Servo tilter = opModeUtilities.getHardwareMap().servo.get("tilter");
        this.tilter = new KServo(tilter);
        this.tilter.setTargetPosition(ModuleConfig.TILTER_SERVO_UP_POS);
    }

    public KServo getTilter() {return tilter;}

    public void setPosition(double target) {
        tilter.setPosition(target);
    }

}
