package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.actions.shooter.pusher.PushBall;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.LaunchPosition;
import com.kalipsorobotics.modules.shooter.Shooter;



public class ShootAllAction extends KActionSet {
    private final static double STOPPER_SERVO_CLOSED_POS = 0.55;
    private final static double STOPPER_SERVO_OPEN_POS = 0.7;

    public ShootAllAction(Stopper stopper, Intake intake, Shooter shooter, Point target, LaunchPosition launchPosition) {
        ShooterReady shooterReady = new ShooterReady(shooter, target, launchPosition);
        shooterReady.setName("ShooterReady");
        this.addAction(shooterReady);

        KServoAutoAction stopperOpen = new KServoAutoAction(stopper.getStopper(), STOPPER_SERVO_CLOSED_POS);
        stopperOpen.setName("stopperOpen");
        stopperOpen.setDependentActions(shooterReady);
        this.addAction(stopperOpen);

        PushBall pushAllBalls = new PushBall(stopper, intake);
        pushAllBalls.setName("pushAllBalls");
        pushAllBalls.setDependentActions(shooterReady, stopperOpen);
        this.addAction(pushAllBalls);

        ShooterStop shooterStop = new ShooterStop(shooter);
        shooterStop.setName("shooterStop");
        shooterStop.setDependentActions(pushAllBalls);
        this.addAction(shooterStop);

    }
}
