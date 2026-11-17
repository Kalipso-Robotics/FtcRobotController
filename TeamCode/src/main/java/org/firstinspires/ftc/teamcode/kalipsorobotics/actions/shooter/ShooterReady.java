package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.SOTM;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig.AUTO_SHOOTER_READY_TIMEOUT_MS;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterReady extends Action {
    public static final String TAG = "Shooting_ShooterReady";

    ShooterRun shooterRun;
    ElapsedTime actionTime;

    public ShooterReady(ShooterRun shooterRun) {
        this.shooterRun = shooterRun;
        this.actionTime = new ElapsedTime();
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
            if (SOTM.isActive()) {
                isDone = true;
            }
            KLog.d(TAG, () -> "name=" + getName() + " event=waiting reason=rpsOutOfRange");
        }

        if (actionTime.milliseconds() > AUTO_SHOOTER_READY_TIMEOUT_MS) {
            isDone = true;
        }

    }

    public ShooterRun getShooterRun() {
        return shooterRun;
    }
}
