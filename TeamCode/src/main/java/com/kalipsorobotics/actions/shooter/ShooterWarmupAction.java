package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.modules.shooter.Shooter;

public class ShooterWarmupAction extends Action {
    Shooter shooter;
    double power;
    public ShooterWarmupAction(Shooter shooter, double power) {
        this.shooter = shooter;
        this.power = power;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        shooter.setPower(this.power);
        isDone = true;
    }
}
