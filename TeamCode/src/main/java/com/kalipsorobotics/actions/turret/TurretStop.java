package com.kalipsorobotics.actions.turret;

import com.kalipsorobotics.actions.actionUtilities.Action;

public class TurretStop extends Action {
    TurretAutoAlignTeleOp turretAutoAlignTeleop;
    public TurretStop(TurretAutoAlignTeleOp turretAutoAlignTeleop) {
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

