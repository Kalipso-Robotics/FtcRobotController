package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig.ACTIVATE_BRAKE_POS;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveBrake;

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
