package com.kalipsorobotics.decode;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.actions.intake.IntakeFullAction;
import com.kalipsorobotics.actions.shooter.ShooterRun;
import com.kalipsorobotics.actions.shooter.ShooterStop;
import com.kalipsorobotics.actions.shooter.pusher.PushBall;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.modules.shooter.ShooterConfig;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.OpModeUtilities;

public class RoundTripAction extends KActionSet {

    private PurePursuitAction moveToBall;

    private IntakeFullAction intakeFullAction;

    private ShooterRun shooterRun;

    public RoundTripAction(OpModeUtilities opModeUtilities, DriveTrain drivetrain, Shooter shooter, Stopper stopper, Intake intake,
                           Point target, Point launchPos, double waitForShooterReadyMS) {
        WaitAction waitUntilShootReady = new WaitAction(waitForShooterReadyMS);
        waitUntilShootReady.setName("waitUntilShootReady");
        this.addAction(waitUntilShootReady);

        // check if this is the correct way to get drivetrain
        PurePursuitAction moveToBalls = new PurePursuitAction(drivetrain);
        this.addAction(moveToBalls);
        this.moveToBall = moveToBalls;
        moveToBalls.setFinalSearchRadius(40);

        //warm - shorter
        shooterRun = new ShooterRun(shooter, target, launchPos, 0);
        shooterRun.setName("shooterReady");
        shooterRun.setDependentActions(waitUntilShootReady);
        this.addAction(shooterRun);

        //
        ShooterRun shooterMaintain = new ShooterRun(shooter, target, launchPos, ShooterConfig.maintainTimeOutMS);
        shooterMaintain.setName("ShooterMaintain");
        this.addAction(shooterMaintain);
        shooterMaintain.setDependentActions(moveToBalls, shooterRun);

        intakeFullAction = new IntakeFullAction(stopper, intake, 10000);
        intakeFullAction.setName("intakeFullAction");
        this.addAction(intakeFullAction);

        PushBall shoot = new PushBall(stopper, intake, shooter);
        shoot.setName("shoot");
        shoot.setDependentActions(intakeFullAction, shooterRun, moveToBalls);
        this.addAction(shoot);

        ShooterStop shooterStop = new ShooterStop(shooterRun);
        shooterStop.setName("shooterStop");
        shooterStop.setDependentActions(shoot, shooterMaintain);
        this.addAction(shooterStop);

    }

    public PurePursuitAction getMoveToBall() {
        return moveToBall;
    }

    @Override
    protected void beforeUpdate() {
        super.beforeUpdate();

        if (moveToBall.getIsDone()){
            KLog.d("intake", "IntakeFullAction set done by pure pursuit");
            KLog.d("RoundTrip", "MoveToBall Done");
            KLog.d("RoundTrip", "Shooter Ready Done" + shooterRun.getIsDone());

            intakeFullAction.setIsDone(true);
        }
    }
}
