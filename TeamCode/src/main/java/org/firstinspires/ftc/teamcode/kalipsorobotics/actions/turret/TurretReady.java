package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Done once the turret is aligned. Leaves TurretAutoAlign running (stop() now switches it to STOP mode,
 * and callers keep aiming through the shot). No SOTM shortcut: a moving shot still waits for the lead angle.
 */
public class TurretReady extends Action {
    public static final String TAG = "Shooting_TurretReady";
    private static final double TURRET_TIMEOUT_MS = 3000; // TODO-measure

    private final TurretAutoAlign turretAutoAlign;
    private final double timeoutMS;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean timedOut = false;

    /** Auto gate: done when aligned, or after 3 s regardless. */
    public TurretReady(TurretAutoAlign turretAutoAlign) {
        this.turretAutoAlign = turretAutoAlign;
        this.timeoutMS = TURRET_TIMEOUT_MS;
    }

    public boolean timedOut() {
        return timedOut;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }

        if (!hasStarted) {
            timer.reset();
            hasStarted = true;
            KLog.d(TAG, () -> "event=start name=" + getName());
        }

        if (turretAutoAlign.isWithinRange()) {
            isDone = true;
            KLog.d(TAG, () -> String.format("event=done name=%s timeMs=%.0f targetDeg=%.1f",
                    getName(), timer.milliseconds(), Math.toDegrees(turretAutoAlign.getTargetAngleRad())));
            return;
        }

        if (!timedOut && timer.milliseconds() > timeoutMS) {
            timedOut = true;
            isDone = true;
            KLog.d(TAG, () -> String.format("event=timeout name=%s timeMs=%.0f deltaDeg=%.1f",
                    getName(), timer.milliseconds(), turretAutoAlign.getDeltaAngleDeg()));
        }
    }
}
