package com.kalipsorobotics.actions.shooter;

import android.graphics.Bitmap;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.shooter.pusher.PushBall;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.LaunchPosition;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.modules.shooter.ShooterConfig;
import com.kalipsorobotics.modules.shooter.ShooterInterpolationConfig;


public class ShootAllAction extends KActionSet {

    public ShootAllAction(Stopper stopper, Intake intake, Shooter shooter, Point target) {
        this(stopper, intake, shooter, target, 0, 0);
    }

    public ShootAllAction(Stopper stopper, Intake intake, Shooter shooter, double targetRPS, double targetHoodPos) {
        this(stopper, intake, shooter, null, targetRPS, targetHoodPos);
    }

    private ShootAllAction(Stopper stopper, Intake intake, Shooter shooter, Point target, double targetRPS, double targetHoodPos) {
        ShooterReady shooterReady = new ShooterReady(shooter, target, LaunchPosition.AUTO, targetRPS, targetHoodPos, 0);
        shooterReady.setName("ShooterReady");
        this.addAction(shooterReady);

        ShooterReady shooterMaintain = new ShooterReady(shooter, target, LaunchPosition.AUTO, targetRPS, targetHoodPos, ShooterConfig.maintainTimeOutMS);
        shooterMaintain.setName("ShooterMaintain");
        this.addAction(shooterMaintain);
        shooterMaintain.setDependentActions(shooterReady);


        PushBall pushAllBalls = new PushBall(stopper, intake, shooter);
        pushAllBalls.setName("pushAllBalls");
        pushAllBalls.setDependentActions(shooterReady);
        this.addAction(pushAllBalls);

        ShooterStop shooterStop = new ShooterStop(shooter);
        shooterStop.setName("shooterStop");
        shooterStop.setDependentActions(pushAllBalls, shooterMaintain);
        this.addAction(shooterStop);

    }
}
