package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.RunUntilStallAction;
import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.modules.Intake;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeRun extends Action {
    Intake intake;
    public IntakeRun(Intake intake) {
        this.intake = intake;
    }

    @Override
    protected void update() {
        intake.getIntakeMotor().setPower(1);
        isDone = true;
    }
}
