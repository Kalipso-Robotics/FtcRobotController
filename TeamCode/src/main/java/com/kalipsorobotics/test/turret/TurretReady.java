package com.kalipsorobotics.test.turret;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.utilities.KLog;

public class TurretReady extends Action {

    private TurretAutoAlign turretAutoAlign;
    public TurretReady(TurretAutoAlign turretAutoAlign) {
        this.turretAutoAlign = turretAutoAlign;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }

        if (!hasStarted) {
            hasStarted = true;
        }

        if (turretAutoAlign.isWithinRange()) {
            isDone = true;
            KLog.d(this.getName(), "TurretReady is done, turret is within range " + turretAutoAlign.getTurret().getTurretMotor().getCurrentPosition() + "ticks, " +
                    "Target Ticks: " + turretAutoAlign.getTargetTicks());
        }
    }
}
