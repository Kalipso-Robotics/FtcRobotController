package com.kalipsorobotics.actions.drivetrain;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.modules.DriveBrake;

public class ReleaseBrakeAction extends KActionSet {
    public ReleaseBrakeAction(DriveBrake driveBrake, ReleaseBraking releaseBraking) {
        releaseBraking = new ReleaseBraking(driveBrake);
    }

    private void generateBasicAction(ReleaseBraking releaseBraking) {
        releaseBraking.setName("ReleaseBraking");
    }
}
