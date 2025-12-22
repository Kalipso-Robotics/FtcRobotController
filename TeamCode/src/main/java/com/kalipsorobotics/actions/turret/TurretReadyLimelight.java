package com.kalipsorobotics.actions.turret;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.actions.turret.TurretAutoAlignLimelight;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretReadyLimelight extends Action {
    private double maxTimeoutMS;
    private ElapsedTime timeoutTimer;
    private static final double DEFAULT_MAX_TIMOUT_MS = 4000;
    private TurretAutoAlignLimelight turretAutoAlignLimelight;
    public TurretReadyLimelight(TurretAutoAlignLimelight turretAutoAlignLimelight) {
        this.turretAutoAlignLimelight = turretAutoAlignLimelight;
        this.maxTimeoutMS = DEFAULT_MAX_TIMOUT_MS;
        timeoutTimer = new ElapsedTime();
    }

    @Override
    protected void update() {
        if (isDone) {
            KLog.d(this.getName(), "time to done: " + timeoutTimer.milliseconds());
            return;
        }

        if (!hasStarted) {
            timeoutTimer.reset();
            hasStarted = true;
            turretAutoAlignLimelight.setIsDone(false);
            KLog.d(this.getName(), "current pos " + SharedData.getOdometryPosition());
        }

        if (timeoutTimer.milliseconds() > maxTimeoutMS) {
            KLog.d(this.getName(), "Turret TIMEOUT in " + maxTimeoutMS + " ms");
            isDone = true;
            return;
        }

        if (turretAutoAlignLimelight.isWithinRange()) {
            isDone = true;
            turretAutoAlignLimelight.setIsDone(true);
            KLog.d(this.getName(), "TurretReady is done, turret is within range " + turretAutoAlignLimelight.getTurret().getTurretMotor().getCurrentPosition() + "ticks, " +
                    "Target Ticks: " + turretAutoAlignLimelight.getTargetTicks());
        }
    }
}
