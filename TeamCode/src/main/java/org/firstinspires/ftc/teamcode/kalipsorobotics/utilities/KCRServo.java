package org.firstinspires.ftc.teamcode.kalipsorobotics.utilities;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class KCRServo {
    private final CRServo crServo;
    private final boolean flipDirection;

    public KCRServo(CRServo servo, boolean flipDirection) {
        this.crServo = servo;
        this.flipDirection = flipDirection;
    }

    public void setPower(double power) {
        power = Math.max(-1.0, Math.min(1.0, power));
        double adjustedPower;
        if (flipDirection) {
            adjustedPower = -power;
        }
        else {
            adjustedPower = power;
        }
        crServo.setPower(adjustedPower);
    }

    public double getPower() {
        return crServo.getPower();
    }

    public CRServo crServo() {
        return crServo;
    }

}
