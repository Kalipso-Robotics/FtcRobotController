package com.kalipsorobotics.modules;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake {

    private final OpModeUtilities opModeUtilities;

    DcMotor intakeMotor;

    public Intake(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        this.intakeMotor = opModeUtilities.getHardwareMap().dcMotor.get("intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public DcMotor getIntakeMotor() {
        return intakeMotor;
    }
}
