package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;

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
