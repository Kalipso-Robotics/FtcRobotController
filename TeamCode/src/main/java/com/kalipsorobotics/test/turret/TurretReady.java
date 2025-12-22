package com.kalipsorobotics.test.turret;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretReady extends Action {
    private double maxTimeoutMS;
    private ElapsedTime timeoutTimer;
    private static final double DEFAULT_MAX_TIMOUT_MS = 4000;
    private TurretAutoAlign turretAutoAlign;
    public TurretReady(TurretAutoAlign turretAutoAlign) {
        this.turretAutoAlign = turretAutoAlign;
        this.maxTimeoutMS = DEFAULT_MAX_TIMOUT_MS;
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
            KLog.d(this.getName(), "current pos " + SharedData.getOdometryIMUPosition());
        }

        if (timeoutTimer.milliseconds() > maxTimeoutMS) {
            KLog.d(this.getName(), "Turret TIMEOUT in " + maxTimeoutMS + " ms");
            isDone = true;
            return;
        }

        if (turretAutoAlign.isWithinRange()) {
            isDone = true;
            KLog.d(this.getName(), "TurretReady is done, turret is within range " + turretAutoAlign.getTurret().getTurretMotor().getCurrentPosition() + "ticks, " +
                    "Target Ticks: " + turretAutoAlign.getTargetTicks());
        }
    }
}
