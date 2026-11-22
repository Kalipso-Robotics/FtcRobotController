package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;

/** Closes the stopper and is done at once (does not wait for the servo). */
public class CloseStopper extends Action {

    private final Stopper stopper;

    public CloseStopper(Stopper stopper) {
        this.stopper = stopper;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        stopper.close(); // through Stopper so the KServo target (isDone, telemetry) stays correct
        setIsDone(true);
    }
}
