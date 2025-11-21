package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.utilities.KLog;

public class ShooterReady extends Action {

    ShooterRun shooterRun;

    public ShooterReady(ShooterRun shooterRun) {
        this.shooterRun = shooterRun;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        KLog.d("ShooterReady", "Shooter Within Range " + shooterRun.isWithinRange());
        isDone = shooterRun.isWithinRange();

    }
}
