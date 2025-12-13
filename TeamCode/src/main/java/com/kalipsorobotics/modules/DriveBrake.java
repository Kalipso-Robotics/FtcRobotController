package com.kalipsorobotics.modules;

import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;

public class DriveBrake {

    private final KServo brakeRight;
    private final KServo brakeLeft;
    //C2 brakeRight  C5 brakeLeft
    public DriveBrake(OpModeUtilities opModeUtilities) {
        this.brakeRight = new KServo(opModeUtilities.getHardwareMap().servo.get("brakeRight"));
        this.brakeLeft = new KServo(opModeUtilities.getHardwareMap().servo.get("brakeLeft"));
    }

    public KServo getBrakeRight() {
        return brakeRight;
    }

    public KServo getBrakeLeft() {
        return brakeLeft;
    }
}
