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
    public TurretReadyTeleOp(TurretAutoAlignTeleOp turretAutoAlignTeleop) {
        this.turretAutoAlignTeleop = turretAutoAlignTeleop;
        this.maxTimeoutMS = DEFAULT_MAX_TIME_OUT_MS;
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
            turretAutoAlignTeleop.updateCheckDone();
            KLog.d(TAG, () -> "event=start name=" + getName());
        }

        if (timeoutTimer.milliseconds() > maxTimeoutMS) {
            KLog.d(TAG, () -> String.format("event=timeout name=%s timeMs=%.0f deltaDeg=%.1f",
                    getName(), timeoutTimer.milliseconds(), turretAutoAlignTeleop.getDeltaAngleDeg()));
            isDone = true;
            return;
        }

        if (turretAutoAlignTeleop.isWithinRange()) {
            isDone = true;
            if (!SOTM.isActive()) {
                turretAutoAlignTeleop.stop();
            }
            KLog.d(TAG, () -> String.format("event=done name=%s timeMs=%.0f targetDeg=%.1f",
                    getName(), timeoutTimer.milliseconds(), Math.toDegrees(turretAutoAlignTeleop.getTargetAngleRad())));
        } else if (SOTM.isActive()) {
            KLog.d(TAG, () -> "event=done name=" + getName() + " reason=sotmActive");
            isDone = true;
        }
    }
}
