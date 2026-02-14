package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretReadyTeleOp extends Action {
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
            KLog.d(this.getName(), "time to done: " + timeoutTimer.milliseconds());
            return;
        }

        if (!hasStarted) {
            timeoutTimer.reset();
            hasStarted = true;
            turretAutoAlignTeleop.updateCheckDone();
            KLog.d(this.getName(), "current pos " + SharedData.getOdometryWheelIMUPosition());
        }

        if (timeoutTimer.milliseconds() > maxTimeoutMS) {
            KLog.d(this.getName(), "Turret TIMEOUT in " + maxTimeoutMS + " ms");
            KLog.d("ActionTime", this.getName() + " done in " + timeoutTimer.milliseconds() + " ms");
            isDone = true;
            return;
        }

        if (turretAutoAlignTeleop.isWithinRange()) {
            isDone = true;
            turretAutoAlignTeleop.stop();
            KLog.d(this.getName(), "TurretReady is done, turret is within range " + turretAutoAlignTeleop.getTurret().getTurretMotor().getCurrentPosition() + "ticks, " +
                    "Target Ticks: " + turretAutoAlignTeleop.getTargetTicks());
            KLog.d("ActionTime", this.getName() + " done in " + timeoutTimer.milliseconds() + " ms");
        } else if (ShooterConfig.shouldShootOnTheMove) {
            isDone = true;
        }
    }
}
