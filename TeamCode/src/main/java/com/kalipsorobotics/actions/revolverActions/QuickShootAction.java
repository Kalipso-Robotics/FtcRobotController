package com.kalipsorobotics.actions.revolverActions;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.actions.shooter.pusher.PushBall;
import com.kalipsorobotics.actions.shooter.ShooterRun;
import com.kalipsorobotics.actions.shooter.ShooterStop;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.modules.shooter.LaunchPosition;
import com.kalipsorobotics.modules.shooter.Shooter;

public class QuickShootAction extends KActionSet {
    public QuickShootAction(Revolver revolver, Shooter shooter, Stopper stopper, Intake intake) {
        ShooterRun shooterRun = new ShooterRun(shooter, Shooter.TARGET_POINT, LaunchPosition.AUTO); //todo set point
        shooterRun.setName("shooterReady");
        this.addAction(shooterRun);

        KServoAutoAction revolverTurn1 = new KServoAutoAction(revolver.getRevolverServo(), Revolver.REVOLVER_INDEX_0);
        revolverTurn1.setName("revolverTurn1");
        revolverTurn1.setDependentActions(shooterRun);
        this.addAction(revolverTurn1);

        WaitAction waitAction1 = new WaitAction(50);
        waitAction1.setName("waitAction1");
        waitAction1.setDependentActions(revolverTurn1);
        this.addAction(waitAction1);

        PushBall pushBall = new PushBall(stopper, intake, shooter);
        pushBall.setName("kickBall");
        pushBall.setDependentActions(revolverTurn1, waitAction1);
        this.addAction(pushBall);

        WaitAction waitAction2 = new WaitAction(1);
        waitAction2.setName("waitAction2");
        waitAction2.setDependentActions(pushBall);
        this.addAction(waitAction2);

        KServoAutoAction revolverTurn2 = new KServoAutoAction(revolver.getRevolverServo(), Revolver.REVOLVER_INDEX_1);
        revolverTurn2.setName("revolverTurn2");
        revolverTurn2.setDependentActions(waitAction2);
        this.addAction(revolverTurn2);

        WaitAction waitAction3 = new WaitAction(50);
        waitAction3.setName("waitAction3");
        waitAction3.setDependentActions(revolverTurn2);
        this.addAction(waitAction3);

        PushBall pushBall2 = new PushBall(stopper, intake, shooter);
        pushBall2.setName("kickBall2");
        pushBall2.setDependentActions(revolverTurn2, waitAction3);
        this.addAction(pushBall2);

        WaitAction waitAction4 = new WaitAction(1);
        waitAction4.setName("waitAction4");
        this.addAction(waitAction4);
        waitAction4.setDependentActions(pushBall2);

        KServoAutoAction revolverTurn3 = new KServoAutoAction(revolver.getRevolverServo(), Revolver.REVOLVER_INDEX_2);
        revolverTurn3.setName("revolverTurn3");
        this.addAction(revolverTurn3);
        revolverTurn3.setDependentActions(waitAction4);

        WaitAction waitAction5 = new WaitAction(50);
        waitAction5.setName("waitAction5");
        this.addAction(waitAction5);
        waitAction5.setDependentActions(revolverTurn3);

        PushBall pushBall3 = new PushBall(stopper, intake, shooter);
        pushBall3.setName("kickBall3");
        pushBall3.setDependentActions(revolverTurn3, waitAction5);
        this.addAction(pushBall3);

        WaitAction waitAction6 = new WaitAction(1);
        waitAction6.setName("waitAction6");
        this.addAction(waitAction6);
        waitAction6.setDependentActions(pushBall3);

        ShooterStop stop = new ShooterStop(shooterRun);
        stop.setName("stop");
        stop.setDependentActions(waitAction6);
        this.addAction(stop);
    }
}
