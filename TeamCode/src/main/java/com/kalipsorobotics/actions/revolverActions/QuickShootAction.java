package com.kalipsorobotics.actions.revolverActions;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.actions.shooter.KickBall;
import com.kalipsorobotics.actions.shooter.ShooterReady;
import com.kalipsorobotics.actions.shooter.ShooterStop;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.modules.shooter.LaunchPosition;
import com.kalipsorobotics.modules.shooter.Shooter;

public class QuickShootAction extends KActionSet {
    public QuickShootAction(Revolver revolver, Shooter shooter) {
        ShooterReady shooterReady = new ShooterReady(shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO); //todo set point
        shooterReady.setName("shooterReady");
        this.addAction(shooterReady);

        KServoAutoAction revolverTurn1 = new KServoAutoAction(revolver.getRevolverServo(), Revolver.REVOLVER_INDEX_0);
        revolverTurn1.setName("revolverTurn1");
        this.addAction(revolverTurn1);

        WaitAction waitAction1 = new WaitAction(200);
        waitAction1.setName("waitAction1");
        this.addAction(waitAction1);
        waitAction1.setDependentActions(revolverTurn1);

        KickBall kickBall = new KickBall(shooter);
        kickBall.setName("kickBall");
        kickBall.setDependentActions(revolverTurn1, waitAction1);
        this.addAction(kickBall);

        WaitAction waitAction2 = new WaitAction(1);
        waitAction2.setName("waitAction2");
        this.addAction(waitAction2);
        waitAction2.setDependentActions(kickBall);

        KServoAutoAction revolverTurn2 = new KServoAutoAction(revolver.getRevolverServo(), Revolver.REVOLVER_INDEX_0);
        revolverTurn2.setName("revolverTurn2");
        this.addAction(revolverTurn2);

        WaitAction waitAction3 = new WaitAction(200);
        waitAction3.setName("waitAction3");
        this.addAction(waitAction3);
        waitAction3.setDependentActions(revolverTurn2);

        KickBall kickBall2 = new KickBall(shooter);
        kickBall2.setName("kickBall2");
        kickBall2.setDependentActions(revolverTurn2, waitAction3);
        this.addAction(kickBall2);

        WaitAction waitAction4 = new WaitAction(1);
        waitAction4.setName("waitAction4");
        this.addAction(waitAction4);
        waitAction4.setDependentActions(kickBall2);

        KServoAutoAction revolverTurn3 = new KServoAutoAction(revolver.getRevolverServo(), Revolver.REVOLVER_INDEX_0);
        revolverTurn3.setName("revolverTurn3");
        this.addAction(revolverTurn3);

        WaitAction waitAction5 = new WaitAction(200);
        waitAction5.setName("waitAction5");
        this.addAction(waitAction5);
        waitAction5.setDependentActions(revolverTurn3);

        KickBall kickBall3 = new KickBall(shooter);
        kickBall3.setName("kickBall3");
        kickBall3.setDependentActions(revolverTurn3, waitAction5);
        this.addAction(kickBall3);

        WaitAction waitAction6 = new WaitAction(1);
        waitAction6.setName("waitAction6");
        this.addAction(waitAction6);
        waitAction6.setDependentActions(kickBall3);

        ShooterStop stop = new ShooterStop(shooter);
        stop.setName("stop");
        stop.setDependentActions(waitAction6);
        this.addAction(stop);
    }
}
