package com.kalipsorobotics.modules;

import static com.kalipsorobotics.decode.configs.ModuleConfig.*;

import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;

public class DriveBrake {

    private final KServo brake;
    //C2 brakeRight  C5 brakeLeft
    public DriveBrake(OpModeUtilities opModeUtilities) {
        this.brake = new KServo(opModeUtilities.getHardwareMap().servo.get("brake")); // port 2
        init();
    }

    private void init() {
        this.brake.setTargetPosition(RELEASE_BRAKE_POS);
    }


    public KServo getBrake() {
        return brake;
    }
}
