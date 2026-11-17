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
            KLog.d(TAG, "Already done, skipping update");
            return;
        }

        if (!hasStarted) {
            actionTime.reset();
            hasStarted = true;
        }

        boolean isWithinRange = shooterRun.isWithinRange();
        KLog.d(TAG, () -> "Checking if shooter within range: " + isWithinRange);
        KLog.d(TAG, () -> "ShooterRun isDone: " + shooterRun.getIsDone());
        if (isWithinRange) {
            isDone = true;
            double currentRPS = shooterRun.getShooter().getRPS();
            double targetRPS = shooterRun.getTargetRPS();
            KLog.d(TAG + "_ShootInfo", () -> "Name: " + this.getName() +
                    " Shooter is within range. Distance:" + shooterRun.getDistanceMM() +
                    " Delta RPS: " + (targetRPS - currentRPS) +
                    " Current RPS: " + currentRPS +
                    " Target RPS: " + targetRPS);
            KLog.d(TAG, "*** SHOOTER READY MARKED AS DONE ***");
            KLog.d(TAG + "_ActionTime", () -> "ActionTime: " + actionTime.milliseconds() + " ms");
            KLog.d(TAG, "*** SHOOTER READY MARKED AS DONE ***");
        } else {
            if (SOTM.isActive()) {
                isDone = true;
            }
            KLog.d(TAG, "** Still waiting for shooter to reach target RPS **");
        }

        if (actionTime.milliseconds() > AUTO_SHOOTER_READY_TIMEOUT_MS) {
            isDone = true;
        }

    }

    public ShooterRun getShooterRun() {
        return shooterRun;
    }
}
