package com.kalipsorobotics.decode;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.actions.intake.IntakeFullAction;
import com.kalipsorobotics.actions.shooter.ShooterReady;
import com.kalipsorobotics.actions.shooter.ShooterStop;
import com.kalipsorobotics.actions.shooter.pusher.PushBall;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.LaunchPosition;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.utilities.OpModeUtilities;

public class RoundTripAction extends KActionSet {

    private PurePursuitAction moveToBall;

    public RoundTripAction(OpModeUtilities opModeUtilities, DriveTrain drivetrain, Shooter shooter, Stopper stopper, Intake intake,
                           Point target, LaunchPosition launch, double waitForShooterReadyMS) {
        WaitAction waitUntilShootReady = new WaitAction(waitForShooterReadyMS);
        waitUntilShootReady.setName("waitUntilShootReady");
        this.addAction(waitUntilShootReady);

        // check if this is the correct way to get drivetrain
        PurePursuitAction moveToBalls = new PurePursuitAction(drivetrain);
        this.addAction(moveToBalls);
        this.moveToBall = moveToBalls;

        ShooterReady ready = new ShooterReady(shooter, target, launch);
        ready.setName("ready");
        ready.setDependentActions(waitUntilShootReady);
        this.addAction(ready);

        IntakeFullAction intakeFullAction = new IntakeFullAction(stopper, intake, 400);
        intakeFullAction.setName("intakeFullAction");
        this.addAction(intakeFullAction);

        PushBall shoot = new PushBall(stopper, intake, shooter);
        shoot.setName("shoot");
        shoot.setDependentActions(moveToBalls);
        this.addAction(shoot);

        ShooterStop stop = new ShooterStop(shooter);
        stop.setName("stop");
        stop.setDependentActions(shoot);
        this.addAction(stop);
    }

    public PurePursuitAction getMoveToBall() {
        return moveToBall;
    }
}
