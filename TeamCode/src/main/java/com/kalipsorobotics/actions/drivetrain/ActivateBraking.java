package com.kalipsorobotics.actions.drivetrain;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.modules.DriveBrake;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.KLog;

public class ActivateBraking extends Action {

    public static final double BRAKE_RIGHT_POS = 0.8449; //0.8135
    public static final double BRAKE_LEFT_POS = 0.663; //0.7195

    private final DriveBrake driveBrake;
    public ActivateBraking(DriveBrake driveBrake) {
        this.driveBrake = driveBrake;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        driveBrake.getBrakeLeft().setTargetPosition(BRAKE_LEFT_POS);
        driveBrake.getBrakeRight().setTargetPosition(BRAKE_RIGHT_POS);
        isDone = true;
    }
}
