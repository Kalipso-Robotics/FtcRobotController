package com.kalipsorobotics.modules;

import com.kalipsorobotics.actions.drivetrain.ActivateBraking;
import com.kalipsorobotics.actions.drivetrain.ReleaseBraking;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;

public class DriveBrake {

    private final KServo brakeRight;
    private final KServo brakeLeft;
    //C2 brakeRight  C5 brakeLeft
    public DriveBrake(OpModeUtilities opModeUtilities) {
        this.brakeRight = new KServo(opModeUtilities.getHardwareMap().servo.get("brakeRight"));
        this.brakeLeft = new KServo(opModeUtilities.getHardwareMap().servo.get("brakeLeft"));
        init();
        KLog.d("Braking", "Braking system initialized");
    }

    private void init() {
        this.brakeLeft.setTargetPosition(ReleaseBraking.RELEASE_BRAKE_LEFT_POS);
        this.brakeRight.setTargetPosition(ReleaseBraking.RELEASE_BRAKE_RIGHT_POS);
    }

    public KServo getBrakeRight() {
        return brakeRight;
    }

    public KServo getBrakeLeft() {
        return brakeLeft;
    }
}
