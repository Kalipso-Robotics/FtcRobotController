package com.kalipsorobotics.actions.turret;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretReadyLimelight extends Action {
    private double maxTimeoutMS;
    private ElapsedTime timeoutTimer;
    private static final double DEFAULT_MAX_TIMOUT_MS = 4000;
    private TurretAutoAlignTeleop turretAutoAlignTeleop;
    public TurretReadyLimelight(TurretAutoAlignTeleop turretAutoAlignTeleop) {
        this.turretAutoAlignTeleop = turretAutoAlignTeleop;
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
            KLog.d(this.getName(), "current pos " + SharedData.getOdometryWheelIMUPosition());
        }

        if (timeoutTimer.milliseconds() > maxTimeoutMS) {
            KLog.d(this.getName(), "Turret TIMEOUT in " + maxTimeoutMS + " ms");
            isDone = true;
            return;
        }

        if (turretAutoAlignTeleop.isWithinRange()) {
            isDone = true;
            turretAutoAlignTeleop.stop();
            KLog.d(this.getName(), "TurretReady is done, turret is within range " + turretAutoAlignTeleop.getTurret().getTurretMotor().getCurrentPosition() + "ticks, " +
                    "Target Ticks: " + turretAutoAlignTeleop.getTargetTicks());
        }
    }
}
