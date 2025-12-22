package com.kalipsorobotics.actions.drivetrain;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.modules.DriveBrake;
import com.kalipsorobotics.utilities.KLog;

public class ReleaseBraking extends Action {

    public static final double RELEASE_BRAKE_RIGHT_POS = 0.7095;
    public static final double RELEASE_BRAKE_LEFT_POS = 0.8199;

    private DriveBrake driveBrake;

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
