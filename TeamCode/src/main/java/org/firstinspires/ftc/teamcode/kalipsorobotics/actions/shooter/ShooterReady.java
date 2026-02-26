package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig.AUTO_SHOOTER_READY_TIMEOUT_MS;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterReady extends Action {

    ShooterRun shooterRun;
    ElapsedTime actionTime;

    public ShooterReady(ShooterRun shooterRun) {
        this.shooterRun = shooterRun;
        this.actionTime = new ElapsedTime();
    }


    @Override
    protected void update() {
        if (isDone) {
            KLog.d("ShooterReady", "Already done, skipping update");
            return;
        }

        if (!hasStarted) {
            actionTime.reset();
            hasStarted = true;
        }

        boolean isWithinRange = shooterRun.isWithinRange();
        KLog.d("ShooterReady", () -> "Checking if shooter within range: " + isWithinRange);
        KLog.d("ShooterReady", () -> "ShooterRun isDone: " + shooterRun.getIsDone());
        if (isWithinRange) {
            isDone = true;
            double currentRPS = shooterRun.getShooter().getRPS();
            double targetRPS = shooterRun.getTargetRPS();
            KLog.d("ShooterReady_ShootInfo", () -> "Name: " + this.getName() +
                    " Shooter is within range. Distance:" + shooterRun.getDistanceMM() +
                    " Delta RPS: " + (targetRPS - currentRPS) +
                    " Current RPS: " + currentRPS +
                    " Target RPS: " + targetRPS);
            KLog.d("ShooterReady", "*** SHOOTER READY MARKED AS DONE ***");
            KLog.d("ShooterReady_ActionTime", () -> "ActionTime: " + actionTime.milliseconds() + " ms");
            KLog.d("ShooterReady", "*** SHOOTER READY MARKED AS DONE ***");
        } else {
            if (ShooterConfig.shouldShootOnTheMoveRPS) {
                isDone = true;
            }
            KLog.d("ShooterReady", "** Still waiting for shooter to reach target RPS **");
        }

        if (actionTime.milliseconds() > AUTO_SHOOTER_READY_TIMEOUT_MS) {
            isDone = true;
        }

    }

    public ShooterRun getShooterRun() {
        return shooterRun;
    }
}
