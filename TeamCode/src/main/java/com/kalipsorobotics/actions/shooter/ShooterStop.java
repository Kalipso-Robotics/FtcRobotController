package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.OpModeUtilities;

public class ShooterStop extends Action {
    ShooterRun shooterRun = null;

    public ShooterStop(ShooterRun shooterRun) {
        this.shooterRun = shooterRun;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        shooterRun.stop();
        isDone = true;
    }
}
