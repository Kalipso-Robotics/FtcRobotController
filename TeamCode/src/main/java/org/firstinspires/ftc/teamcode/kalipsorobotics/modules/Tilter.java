package org.firstinspires.ftc.teamcode.kalipsorobotics.modules;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.Servo;

public class Tilter {

    private final OpModeUtilities opModeUtilities;

    private final KServo tilterRight;

    private final KServo tilterLeft;

    public Tilter(OpModeUtilities opModeUtilities) {
        KLog.d("Tilter", "Constructor called - starting initialization");
        this.opModeUtilities = opModeUtilities;
        Servo tilterLeft = opModeUtilities.getHardwareMap().servo.get("tilterLeft");
        this.tilterLeft = new KServo(tilterLeft);
        this.tilterLeft.setTargetPosition(ModuleConfig.TILT_LEFT_UP_POS);

        Servo tilterRight = opModeUtilities.getHardwareMap().servo.get("tilterRight");
        this.tilterRight = new KServo(tilterRight);
        this.tilterRight.setTargetPosition(ModuleConfig.TILT_LEFT_UP_POS);
    }

    public KServo getTilterLeft() {return tilterLeft;}
    public void setTilterLeftPosition(double target) {
        tilterLeft.setPosition(target);
    }

    public KServo getTilterRight() {return tilterRight;}
    public void setTilterRightPosition(double target) {
        tilterRight.setPosition(target);
    }



}
