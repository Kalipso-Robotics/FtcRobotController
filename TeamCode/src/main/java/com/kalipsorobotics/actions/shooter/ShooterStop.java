package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.OpModeUtilities;

public class ShooterStop extends Action {
    Shooter shooter = null;

    public ShooterStop(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    protected void update() {
        shooter.stop();
        isDone = true;
    }
}
