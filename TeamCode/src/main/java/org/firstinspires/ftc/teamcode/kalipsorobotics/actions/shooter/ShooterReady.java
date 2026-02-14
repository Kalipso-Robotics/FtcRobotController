package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter;

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
        KLog.d("ShooterReady", "Checking if shooter within range: " + isWithinRange);
        KLog.d("ShooterReady", "ShooterRun isDone: " + shooterRun.getIsDone());
        if (isWithinRange) {
            isDone = true;
            KLog.d("ShooterReady_ShootInfo", "Name: " + this.getName() +
                    " Shooter is within range. Distance:" + shooterRun.getDistanceMM() +
                    " Current RPS: " + shooterRun.getShooter().getRPS() +
                    " Target RPS: " + shooterRun.getTargetRPS());
            KLog.d("ShooterReady", "*** SHOOTER READY MARKED AS DONE ***");
            KLog.d("ShooterReady_ActionTime", "ActionTime: " + actionTime.milliseconds() + " ms");
            KLog.d("ShooterReady", "*** SHOOTER READY MARKED AS DONE ***");
        } else {
            if (ShooterConfig.shouldShootOnTheMove) {
                isDone = true;
            }
            KLog.d("ShooterReady", "** Still waiting for shooter to reach target RPS **");
        }

    }

    public ShooterRun getShooterRun() {
        return shooterRun;
    }
}
