package com.kalipsorobotics.actions.drivetrain;

import static com.kalipsorobotics.decode.configs.ModuleConfig.ACTIVATE_BRAKE_POS;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.modules.DriveBrake;

public class ActivateBraking extends Action {


    private final DriveBrake driveBrake;
    public ActivateBraking(DriveBrake driveBrake) {
        this.driveBrake = driveBrake;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        driveBrake.getBrake().setTargetPosition(ACTIVATE_BRAKE_POS);
        isDone = true;
    }
}
