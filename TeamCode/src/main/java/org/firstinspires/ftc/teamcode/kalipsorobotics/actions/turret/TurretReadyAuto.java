package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretReadyAuto extends Action {
    public static final String TAG = "Shooting_TurretReady";

    private final double maxTimeoutMS;
    private final ElapsedTime timeoutTimer;
    private static final double DEFAULT_MAX_TIMEOUT_MS = 3000;
    private final TurretAutoAlign turretAutoAlign;
    public TurretReadyAuto(TurretAutoAlign turretAutoAlign) {
        this.turretAutoAlign = turretAutoAlign;
        this.maxTimeoutMS = DEFAULT_MAX_TIMEOUT_MS;
        timeoutTimer = new ElapsedTime();
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }

        if (!hasStarted) {
            timeoutTimer.reset();
            hasStarted = true;
            KLog.d(TAG, () -> "event=start name=" + getName());
        }

        if (timeoutTimer.milliseconds() > maxTimeoutMS) {
            KLog.d(TAG, () -> String.format("event=timeout name=%s timeMs=%.0f deltaDeg=%.1f",
                    getName(), timeoutTimer.milliseconds(), turretAutoAlign.getDeltaAngleDeg()));
            isDone = true;
            return;
        }

        if (turretAutoAlign.isWithinRange()) {
            isDone = true;
            KLog.d(TAG, () -> String.format("event=done name=%s timeMs=%.0f targetDeg=%.1f",
                    getName(), timeoutTimer.milliseconds(), Math.toDegrees(turretAutoAlign.getTargetAngleRad())));
        }
    }
}
