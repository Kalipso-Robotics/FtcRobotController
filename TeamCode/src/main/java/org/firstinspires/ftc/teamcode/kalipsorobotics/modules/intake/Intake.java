package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake {

    private final OpModeUtilities opModeUtilities;

    DcMotor intakeMotor;

    public Intake(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        this.intakeMotor = opModeUtilities.getHardwareMap().dcMotor.get("intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public DcMotor getIntakeMotor() {
        return intakeMotor;
    }

    public DcMotorEx getIntakeMotorEx() {
        return (DcMotorEx) intakeMotor;
    }
}
