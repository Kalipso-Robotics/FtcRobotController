package com.kalipsorobotics.actions.shooter.pusher;


import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.actions.intake.RunIntakeUntilFullSpeed;
import com.kalipsorobotics.actions.shooter.AdjustShooterSpeedAction;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.LaunchPosition;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.KLog;

public class PushBall extends KActionSet {

    public PushBall(Stopper stopper, Intake intake, Shooter shooter) {
        KServoAutoAction release = new KServoAutoAction(stopper.getStopper(), stopper.STOPPER_SERVO_OPEN_POS);
        release.setName("release");
        this.addAction(release);

        RunIntakeUntilFullSpeed untilShootingDone = new RunIntakeUntilFullSpeed(intake);
        untilShootingDone.setName("untilShootingDone");
//        untilShootingDone.setDependentActions(pushLeft, pushRight);
        this.addAction(untilShootingDone);

        AdjustShooterSpeedAction check1 = new AdjustShooterSpeedAction(shooter);
        check1.setName("check1");
        check1.setDependentActions(release);
        this.addAction(check1);

        KLog.d("teleop", "shooting done.(based off intake)");
        WaitAction wait = new WaitAction(1000);
        wait.setDependentActions(untilShootingDone);
        this.addAction(wait);

//        KServoAutoAction block = new KServoAutoAction(stopper.getStopper(), stopper.STOPPER_SERVO_CLOSED_POS);
//        block.setName("block");
//        this.addAction(block);
//        block.setDependentActions(wait, untilShootingDone, release);


        KLog.d("teleop", "pusher Stopped(based off intake)");

    }

}
