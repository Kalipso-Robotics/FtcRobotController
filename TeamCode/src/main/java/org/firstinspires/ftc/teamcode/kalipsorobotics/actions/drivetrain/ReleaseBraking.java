package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig.*;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveBrake;

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
        driveBrake.getBrake().setTargetPosition(RELEASE_BRAKE_POS);
        isDone = true;
    }
}
