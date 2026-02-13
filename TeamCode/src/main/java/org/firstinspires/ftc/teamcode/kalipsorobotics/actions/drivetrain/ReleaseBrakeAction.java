package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveBrake;

public class ReleaseBrakeAction extends KActionSet {
    public ReleaseBrakeAction(DriveBrake driveBrake, ReleaseBraking releaseBraking) {
        releaseBraking = new ReleaseBraking(driveBrake);

        generateBasicAction(releaseBraking);
    }

    private void generateBasicAction(ReleaseBraking releaseBraking) {
        releaseBraking.setName("ReleaseBraking");
        this.addAction(releaseBraking);
    }
}
