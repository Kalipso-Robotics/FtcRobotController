package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Done once the TurretAutoAlign is aligned (auto and TeleOp). The timer starts on the first update.
 * On timeout: by default it is done anyway (auto shoots rather than stall); with holdOnTimeout it only sets
 * timedOut() and stays not done, so ShootAllAction can abort before feeding (TeleOp).
 */
public class TurretReady extends Action {
    public static final String TAG = "Shooting_TurretReady";

    private static final double DEFAULT_TIMEOUT_MS = 3000;

    private final TurretAutoAlign turretAutoAlign;
    private final double timeoutMS;
    private final boolean holdOnTimeout;
    private final ElapsedTime timeoutTimer = new ElapsedTime();
    private boolean timedOut = false;

    public TurretReady(TurretAutoAlign turretAutoAlign) {
        this(turretAutoAlign, DEFAULT_TIMEOUT_MS, false);
    }

    public TurretReady(TurretAutoAlign turretAutoAlign, double timeoutMS, boolean holdOnTimeout) {
        this.turretAutoAlign = turretAutoAlign;
        this.timeoutMS = timeoutMS;
        this.holdOnTimeout = holdOnTimeout;
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
            hasStarted = true;
            timeoutTimer.reset();
            turretAutoAlign.updateCheckDone(); // fresh isWithinRange() for the current target
            KLog.d(TAG, () -> "event=start name=" + getName());
        }

        // A moving shot (SOTM) still waits for the turret to reach the lead angle; the timeout covers it never settling.
        if (turretAutoAlign.isWithinRange()) {
            isDone = true;
            KLog.d(TAG, () -> String.format("event=done name=%s timeMs=%.0f targetDeg=%.1f",
                    getName(), timeoutTimer.milliseconds(), Math.toDegrees(turretAutoAlign.getTargetAngleRad())));
            return;
        }

        if (!timedOut && timeoutTimer.milliseconds() > timeoutMS) {
            timedOut = true;
            KLog.d(TAG, () -> String.format("event=timeout name=%s timeMs=%.0f deltaDeg=%.1f",
                    getName(), timeoutTimer.milliseconds(), turretAutoAlign.getDeltaAngleDeg()));
            if (!holdOnTimeout) {
                isDone = true;
            }
        }
    }
}
