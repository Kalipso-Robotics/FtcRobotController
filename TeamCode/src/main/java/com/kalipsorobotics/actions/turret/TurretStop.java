package com.kalipsorobotics.actions.turret;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.modules.Turret;

public class TurretStop extends Action {
    TurretAutoAlignLimelight turretAutoAlignLimelight;
    public TurretStop(TurretAutoAlignLimelight turretAutoAlignLimelight) {
        this.turretAutoAlignLimelight = turretAutoAlignLimelight;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }

        turretAutoAlignLimelight.stopAndSetDone();
        isDone = true;
    }


}

