package com.kalipsorobotics.actions.turret;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.modules.Turret;

public class TurretMove extends Action {
    Turret turret;
    double power;
    public TurretMove(Turret turret, double power) {
        this.turret = turret;
        this.power = power;
    }

    @Override
    protected void update() {
        turret.getTurretMotor().setPower(power);
        isDone = true;
    }
}
