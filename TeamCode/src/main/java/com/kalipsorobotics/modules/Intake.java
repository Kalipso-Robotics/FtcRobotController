package com.kalipsorobotics.modules;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {

    private final OpModeUtilities opModeUtilities;

    DcMotor intakeMotor;

    public Intake(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        this.intakeMotor = opModeUtilities.getHardwareMap().dcMotor.get("intakeMotor");
    }

    public DcMotor getIntakeMotor() {
        return intakeMotor;
    }
}
