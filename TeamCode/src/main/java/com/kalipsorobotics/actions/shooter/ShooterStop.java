package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.modules.shooter.ShooterInterpolationConfig;
import com.kalipsorobotics.utilities.OpModeUtilities;

public class ShooterStop extends Action {
    Shooter shooter;
    ShootAllAction shootAllAction;

    ShooterRun shooterRun;

    public ShooterStop(ShooterRun shooterRun) {
        this.shooterRun = shooterRun;
        this.shooter = shooterRun.getShooter();
    }

    public ShooterStop(Shooter shooter, ShootAllAction shootAllAction) {
        this.shooter = shooter;
        this.shootAllAction = shootAllAction;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        shooter.setPower(0);
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
