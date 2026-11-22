package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

/**
 * Points the turret at a fixed angle (deg, robot frame, clamped by Turret) and is done when it stops moving.
 * Use it to pre-aim before the start or the park.
 * Auto: {@code new TurretSetAngle(turret, 0)}
 * Pattern source: TurretReady (done on !turret.isMoving()).
 */
public class TurretSetAngle extends Action {
    public static final String TAG = "Shooting_TurretSetAngle";

    private final Turret turret;
    private final double angleDeg;

    public TurretSetAngle(Turret turret, double angleDeg) {
        this.turret = turret;
        this.angleDeg = angleDeg;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        if (!hasStarted) {
            hasStarted = true;
            turret.setAngleDeg(angleDeg);
            KLog.d(TAG, () -> "name=" + getName() + " event=start angleDeg=" + angleDeg);
            return; // isMoving() is set by setAngleDeg; check from the next loop
        }
        if (!turret.isMoving()) {
            isDone = true;
            KLog.d(TAG, () -> "name=" + getName() + " event=done angleDeg=" + turret.getAngleDeg());
        }
    }
}
