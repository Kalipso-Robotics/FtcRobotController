package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretReadyAuto extends Action {
    private final double maxTimeoutMS;
    private final ElapsedTime timeoutTimer;
    private static final double DEFAULT_MAX_TIMEOUT_MS = 4000;
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
            KLog.d(this.getName(), "current pos " + SharedData.getOdometryWheelIMUPosition());
        }

        if (timeoutTimer.milliseconds() > maxTimeoutMS) {
            KLog.d(this.getName(), "Turret TIMEOUT in " + maxTimeoutMS + " ms");
            KLog.d("ActionTime", this.getName() + " timeout in " + timeoutTimer.milliseconds() + " ms");
            isDone = true;
            return;
        }

        if (turretAutoAlign.isWithinRange()) {
            isDone = true;
            KLog.d("ActionTime", this.getName() + " done in " + timeoutTimer.milliseconds() + " ms");
            KLog.d(this.getName(), "Turret took " + timeoutTimer.milliseconds() + " milliseconds");
            KLog.d(this.getName(), "TurretReady is done, turret is within range " + turretAutoAlign.getTurret().getTurretMotor().getCurrentPosition() + "ticks, " +
                    "Target Ticks: " + turretAutoAlign.getTargetTicks());
        }
    }
}
