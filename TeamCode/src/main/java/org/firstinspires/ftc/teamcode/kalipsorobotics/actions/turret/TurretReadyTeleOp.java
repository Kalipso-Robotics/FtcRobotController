package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.SOTM;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretReadyTeleOp extends Action {
    public static final String TAG = TurretReadyAuto.TAG;

    private final double maxTimeoutMS;
    private final ElapsedTime timeoutTimer;
    private static final double DEFAULT_MAX_TIME_OUT_MS = 4000;
    private final TurretAutoAlignTeleOp turretAutoAlignTeleop;
    private final boolean holdOnTimeout;
    private boolean timedOut = false;

    public TurretReadyTeleOp(TurretAutoAlignTeleOp turretAutoAlignTeleop) {
        this.turretAutoAlignTeleop = turretAutoAlignTeleop;
        this.maxTimeoutMS = DEFAULT_MAX_TIME_OUT_MS;
        this.holdOnTimeout = false;
        timeoutTimer = new ElapsedTime();
    }

    /** TeleOp gate: on timeout it only sets timedOut() and stays not-done, so ShootAllAction can abort before feeding. */
    public TurretReadyTeleOp(TurretAutoAlignTeleOp turretAutoAlignTeleop, double timeoutMS) {
        this.turretAutoAlignTeleop = turretAutoAlignTeleop;
        this.maxTimeoutMS = timeoutMS;
        this.holdOnTimeout = true;
        timeoutTimer = new ElapsedTime();
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
            timeoutTimer.reset();
            hasStarted = true;
            turretAutoAlignTeleop.updateCheckDone();
            KLog.d(TAG, () -> "event=start name=" + getName());
        }

        if (timeoutTimer.milliseconds() > maxTimeoutMS) {
            KLog.d(TAG, () -> String.format("event=timeout name=%s timeMs=%.0f deltaDeg=%.1f",
                    getName(), timeoutTimer.milliseconds(), turretAutoAlignTeleop.getDeltaAngleDeg()));
            timedOut = true;
            if (!holdOnTimeout) {
                isDone = true;
                return;
            }
        }

        if (turretAutoAlignTeleop.isWithinRange()) {
            isDone = true;
            if (!SOTM.isActive()) {
                turretAutoAlignTeleop.stop();
            }
            KLog.d(TAG, () -> String.format("event=done name=%s timeMs=%.0f targetDeg=%.1f",
                    getName(), timeoutTimer.milliseconds(), Math.toDegrees(turretAutoAlignTeleop.getTargetAngleRad())));
        }
        // No SOTM shortcut: a moving shot still waits for the turret to reach the lead angle (it tracks a slowly
        // moving target, so isWithinRange() still settles; the TeleOp timeout aborts if it never does).
    }
}
