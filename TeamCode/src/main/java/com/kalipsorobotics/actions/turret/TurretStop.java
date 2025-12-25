package com.kalipsorobotics.actions.turret;

import com.kalipsorobotics.actions.actionUtilities.Action;

public class TurretStop extends Action {
    TurretAutoAlignTeleop turretAutoAlignTeleop;
    public TurretStop(TurretAutoAlignTeleop turretAutoAlignTeleop) {
        this.turretAutoAlignTeleop = turretAutoAlignTeleop;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        turretAutoAlignTeleop.stop();
        isDone = true;
    }


}

