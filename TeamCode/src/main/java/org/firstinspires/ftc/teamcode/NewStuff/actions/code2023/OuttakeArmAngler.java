package org.firstinspires.ftc.teamcode.NewStuff.actions.code2023;

import static java.lang.Math.toRadians;

import android.graphics.Path;

import com.kuriosityrobotics.shuttle.hardware.ServoControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NewStuff.OpModeUtilities;

public class OuttakeArmAngler extends ServoControl {
    private static final double ARM_SERVO_SPEED = toRadians(240); // per second
    private static final double ARM_SERVO_RANGE = toRadians(300);
    public static final boolean FLIP_DIRECTION = false;
    public static final double ZERO_POSITION = toRadians(150);

    private OuttakeArmAngler(Servo servo) {
        super(servo, ARM_SERVO_SPEED, ARM_SERVO_RANGE, FLIP_DIRECTION, ZERO_POSITION);
    }

    public OuttakeArmAngler(OpModeUtilities opModeUtilities) {
        this(opModeUtilities.getHardwareMap().servo.get("trayAngle"));
    }

    public void goToRight() throws InterruptedException {
        goToAngle(toRadians(-30));
    }
    public void goToLeft() throws InterruptedException {
        goToAngle(toRadians(30));
    }
    public void goToDefault() throws InterruptedException {
        goToAngle(toRadians(0));
    }
}