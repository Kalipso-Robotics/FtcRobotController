package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig.AUTO_SHOOTER_READY_TIMEOUT_MS;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterReady extends Action {
    public static final String TAG = "Shooting_ShooterReady";

    ShooterRun shooterRun;
    ElapsedTime actionTime;
    private final double timeoutMS;
    private final boolean holdOnTimeout;
    private boolean timedOut = false;

    public ShooterReady(ShooterRun shooterRun) {
        this.shooterRun = shooterRun;
        this.actionTime = new ElapsedTime();
        this.timeoutMS = AUTO_SHOOTER_READY_TIMEOUT_MS;
        this.holdOnTimeout = false;
    }

    /** TeleOp gate: on timeout it only sets timedOut() and stays not-done, so ShootAllAction can abort before feeding. */
    public ShooterReady(ShooterRun shooterRun, double timeoutMS) {
        this.shooterRun = shooterRun;
        this.actionTime = new ElapsedTime();
        this.timeoutMS = timeoutMS;
        this.holdOnTimeout = true;
    }

    public boolean timedOut() {
        return timedOut;
    }


    @Override
    protected void update() {
        if (isDone) {
            KLog.d(TAG, () -> "name=" + getName() + " event=skip reason=alreadyDone");
            return;
        }

        if (!hasStarted) {
            actionTime.reset();
            hasStarted = true;
        }

        boolean isWithinRange = shooterRun.isWithinRange();
        KLog.d(TAG, () -> "name=" + getName() + " withinRange=" + isWithinRange + " shooterRunDone=" + shooterRun.getIsDone());
        if (isWithinRange) {
            isDone = true;
            double currentRPS = shooterRun.getShooter().getRPS();
            double targetRPS = shooterRun.getTargetRPS();
            KLog.d(TAG, () -> "name=" + getName() + " event=ready distanceMm=" + shooterRun.getDistanceMM()
                    + " deltaRps=" + (targetRPS - currentRPS) + " rps=" + currentRPS + " targetRps=" + targetRPS
                    + " elapsedMs=" + actionTime.milliseconds());
        } else {
            KLog.d(TAG, () -> "name=" + getName() + " event=waiting reason=rpsOutOfRange");
        }

        // Not when it just became ready: that is not a timeout.
        if (!isDone && actionTime.milliseconds() > timeoutMS) {
            timedOut = true;
            if (!holdOnTimeout) {
                isDone = true;
            }
        }

    }

    public ShooterRun getShooterRun() {
        return shooterRun;
    }
}
