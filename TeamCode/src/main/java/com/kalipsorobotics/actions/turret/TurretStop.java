package com.kalipsorobotics.actions.turret;

import com.kalipsorobotics.actions.actionUtilities.Action;

public class TurretStop extends Action {
    TurretAutoAlignTeleOp turretAutoAlignTeleop = null;
    TurretAutoAlign turretAutoAlign = null;
    public TurretStop(TurretAutoAlignTeleOp turretAutoAlignTeleop) {
        this.turretAutoAlignTeleop = turretAutoAlignTeleop;
    }
    public TurretStop(TurretAutoAlign turretAutoAlign) {
        this.turretAutoAlign = turretAutoAlign;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        if (turretAutoAlignTeleop != null) {
            turretAutoAlignTeleop.stop();
        }
        if (turretAutoAlign != null){
            turretAutoAlign.stop();
        }
        isDone = true;
    }


}

