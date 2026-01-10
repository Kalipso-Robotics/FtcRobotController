package com.kalipsorobotics.actions.drivetrain;

import static com.kalipsorobotics.decode.configs.ModuleConfig.*;
import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.modules.DriveBrake;

public class ReleaseBraking extends Action {


    private final DriveBrake driveBrake;

    public ReleaseBraking(DriveBrake driveBrake) {
        this.driveBrake = driveBrake;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        driveBrake.getBrakeLeft().setTargetPosition(RELEASE_BRAKE_LEFT_POS);
        driveBrake.getBrakeRight().setTargetPosition(RELEASE_BRAKE_RIGHT_POS);
        isDone = true;
    }
}
