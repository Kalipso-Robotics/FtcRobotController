package com.kalipsorobotics.fresh.mechanisms;

import static java.lang.Math.toRadians;

import com.kalipsorobotics.fresh.OpModeUtilities;
import com.kuriosityrobotics.shuttle.hardware.ServoControl;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeArmClamp extends ServoControl {
    private static final double ARM_SERVO_SPEED = toRadians(240); // per second
    private static final double ARM_SERVO_RANGE = toRadians(300);
    public static final boolean FLIP_DIRECTION = true;
    public static final double ZERO_POSITION = toRadians(171);

    private OuttakeArmClamp(Servo servo) {
        super(servo, ARM_SERVO_SPEED, ARM_SERVO_RANGE, FLIP_DIRECTION, ZERO_POSITION);
    }
    public OuttakeArmClamp(OpModeUtilities opModeUtilities) {
        this(opModeUtilities.getHardwareMap().servo.get("holderClamp"));
    }

    public void release() throws InterruptedException {
        goToAngle(toRadians(29.7));
    }
    public void clamp() throws InterruptedException {
        goToAngle(toRadians(0));
    }
}
