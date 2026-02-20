package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

public class ShooterStop extends Action {
    Shooter shooter;
    ShootAllAction shootAllAction;

    ShooterRun shooterRun;


    public ShooterStop(ShooterRun shooterRun) {
        this.shooterRun = shooterRun;
        this.shooter = shooterRun.getShooter();
    }

    public ShooterStop(Shooter shooter, ShootAllAction shootAllAction) {
        this.shooter = shooter;
        this.shootAllAction = shootAllAction;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        KLog.d("ShooterStop", () -> "Stopping shooter");
        shooter.setPower(0);
        if (shooterRun != null) {
            shooterRun.stop();
            shooterRun.setIsDone(true);
        }
        if (shootAllAction != null) {
            shootAllAction.getShooterRun().stop();
            shootAllAction.setIsDone(true);
        }
        isDone = true;
    }
}
