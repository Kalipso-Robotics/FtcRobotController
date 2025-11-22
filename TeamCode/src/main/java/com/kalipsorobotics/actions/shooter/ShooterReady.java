package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.utilities.KLog;

public class ShooterReady extends Action {

    ShooterRun shooterRun;

    public ShooterReady(ShooterRun shooterRun) {
        this.shooterRun = shooterRun;
    }

    @Override
    public boolean updateCheckDone() {
        if (isDone) {
            return true;
        }

        // Check dependencies
        for (Action action : dependentActions) {
            if (action != null && !action.getIsDone()) {
                KLog.d("ShooterReady", "Waiting for dependent action: " + action.getName() + " (isDone: " + action.getIsDone() + ")");
                return false;
            }
        }

        KLog.d("ShooterReady", "All dependencies done, calling update()");
        update();
        return isUpdateDone();
    }

    @Override
    protected void update() {
        if (isDone) {
            KLog.d("ShooterReady", "Already done, skipping update");
            return;
        }
        boolean isWithinRange = shooterRun.isWithinRange();
        KLog.d("ShooterReady", "Checking if shooter within range: " + isWithinRange);
        KLog.d("ShooterReady", "ShooterRun isDone: " + shooterRun.getIsDone());
        if (isWithinRange) {
            isDone = true;
            KLog.d("ShooterReady", "*** SHOOTER READY MARKED AS DONE ***");
        } else {
            KLog.d("ShooterReady", "Still waiting for shooter to reach target RPS");
        }

    }
}
