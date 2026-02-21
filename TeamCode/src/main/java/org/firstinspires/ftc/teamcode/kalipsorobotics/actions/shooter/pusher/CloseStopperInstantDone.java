package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;

public class CloseStopperInstantDone extends Action {

    private final Stopper stopper;

    public CloseStopperInstantDone(Stopper stopper) {
        this.stopper = stopper;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        stopper.getStopper().setPosition(ModuleConfig.STOPPER_SERVO_CLOSED_POS);
        setIsDone(true);
    }
}
