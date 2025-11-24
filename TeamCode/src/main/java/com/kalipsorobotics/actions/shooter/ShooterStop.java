package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.OpModeUtilities;

public class ShooterStop extends Action {
    ShooterRun shooterRun;
    ShootAllAction shootAllAction;

    public ShooterStop(ShooterRun shooterRun) {
        this.shooterRun = shooterRun;
    }

    public ShooterStop(ShooterRun shooterRun, ShootAllAction shootAllAction) {
        this.shooterRun = shooterRun;
        this.shootAllAction = shootAllAction;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        if (shooterRun != null) {
            shooterRun.stop();
            shooterRun.setIsDone(true);
        }
        if (shootAllAction != null) {
            shootAllAction.getShooterRun().stop();
            shootAllAction.setIsDone(true);
        }
        isDone = true;
    }
}
