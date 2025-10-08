package com.kalipsorobotics.actions.revolverActions;

import com.kalipsorobotics.PID.OpMode;
import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.Gamepad;

public class RevolverTeleop extends Action {

    Revolver revolver;
    Gamepad gamepad1;
    double revolverPosition;

    public RevolverTeleop(OpModeUtilities opMode) {
        this.gamepad1 = opMode.getOpMode().gamepad1;
    }

    @Override
    protected boolean checkDoneCondition() {
        return false;
    }

    @Override
    protected void update() {
        if (gamepad1.left_stick_x > 0) {
            revolver.revolverServo.setPosition(revolverPosition);
            revolverPosition += 0.01;
        } else if (gamepad1.left_stick_x < 0) {
            revolver.revolverServo.setPosition(revolverPosition);
            revolverPosition -= 0.01;
        }
    }
}
