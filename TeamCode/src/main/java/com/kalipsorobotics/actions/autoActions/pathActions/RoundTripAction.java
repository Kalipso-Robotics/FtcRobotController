package com.kalipsorobotics.actions.autoActions.pathActions;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.actions.intake.IntakeFullAction;
import com.kalipsorobotics.actions.shooter.ShooterReady;
import com.kalipsorobotics.actions.shooter.ShooterRun;
import com.kalipsorobotics.actions.shooter.ShooterStop;
import com.kalipsorobotics.actions.shooter.pusher.PushBall;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;

public class RoundTripAction extends KActionSet {

    private PurePursuitAction moveToBall;

    private IntakeFullAction intakeFullAction;

    private ShooterRun shooterRun;
    private Shooter shooter;
    private ShooterReady shooterReady;
    private PushBall shoot;
    private ShooterStop shooterStop;
    private boolean hasUpdatedShooterReady = false;

    Point targetPoint;

    public RoundTripAction(OpModeUtilities opModeUtilities, DriveTrain drivetrain, Shooter shooter, Stopper stopper, Intake intake,
                           Point targetPoint, Point launchPoint, double waitForShooterReadyMS, boolean shouldRunIntake) {
        this.targetPoint = targetPoint;
        this.shooter = shooter;

        WaitAction waitUntilShootRun = new WaitAction(waitForShooterReadyMS);
        waitUntilShootRun.setName("waitUntilShootReady");
        this.addAction(waitUntilShootRun);

        // check if this is the correct way to get drivetrain
        PurePursuitAction moveToBalls = new PurePursuitAction(drivetrain);
        moveToBalls.setName("moveToBall");  // FIX: Name the action!
        this.addAction(moveToBalls);
        this.moveToBall = moveToBalls;
        moveToBalls.setFinalSearchRadius(100);

        //warm - shorter
        shooterRun = new ShooterRun(shooter, targetPoint, launchPoint);
        shooterRun.setName("shooterRun");  // FIX: Correct name (was "shooterReady")
        shooterRun.setDependentActions(waitUntilShootRun);
        this.addAction(shooterRun);

        shooterReady = new ShooterReady(shooterRun);
        shooterReady.setName("shooterReady");
        shooterReady.setDependentActions(moveToBalls);
        this.addAction(shooterReady);

        intakeFullAction = new IntakeFullAction(stopper, intake, 8000);
        intakeFullAction.setName("intakeFullAction");
        this.addAction(intakeFullAction);

        if (!shouldRunIntake) {
            intakeFullAction.setIsDone(true);
        }

        shoot = new PushBall(stopper, intake, shooter);
        shoot.setName("shoot");
        shoot.setDependentActions(shooterReady, moveToBalls);
        this.addAction(shoot);

        shooterStop = new ShooterStop(shooterRun);
        shooterStop.setName("stop");
        shooterStop.setDependentActions(shoot);
        this.addAction(shooterStop);

    }

    public RoundTripAction(OpModeUtilities opModeUtilities, DriveTrain drivetrain, Shooter shooter, Stopper stopper, Intake intake,
                           Point targetPoint, Point launchPos, double waitForShooterReadyMS) {
        this(opModeUtilities, drivetrain, shooter, stopper, intake, targetPoint, launchPos, waitForShooterReadyMS, true);
    }

    public PurePursuitAction getMoveToBall() {
        return moveToBall;
    }

    @Override
    protected void beforeUpdate() {
        super.beforeUpdate();

        KLog.d("RoundTrip", String.format("[%s] Status - MoveToBall: %s, Intake: %s, ShooterReady %s, ShooterRun: %s, PushBall: %s, ShooterStop: %s, ShooterReadyUpdated: %b",
            getName() != null ? getName() : "unnamed",
            moveToBall.getIsDone() ? "DONE" : "NOT DONE",
            intakeFullAction.getIsDone() ? "DONE" : "NOT DONE",
            shooterReady.getIsDone() ? "DONE" : "NOT DONE",
            shooterRun.getIsDone() ? "DONE" : "NOT DONE",
            shoot.getIsDone() ? "DONE" : "NOT DONE",
            shooterStop.getIsDone() ? "DONE" : "NOT DONE",
            hasUpdatedShooterReady));

        if (moveToBall.getIsDone()){
            KLog.d("RoundTrip", String.format("[%s] MoveToBall COMPLETED - Stopping intake and updating shooter position",
                getName() != null ? getName() : "unnamed"));

            intakeFullAction.stop();
            KLog.d("RoundTrip", String.format("[%s] IntakeFullAction stopped by pure pursuit completion",
                getName() != null ? getName() : "unnamed"));

            if (!hasUpdatedShooterReady) {
                Point currentPos = new Position(SharedData.getOdometryPosition()).toPoint();
                KLog.d("RoundTrip", String.format("[%s] Updating shooter position - Current: (%.1f, %.1f), Target: (%.1f, %.1f)",
                    getName() != null ? getName() : "unnamed",
                    currentPos.getX(), currentPos.getY(),
                    targetPoint.getX(), targetPoint.getY()));
//                shooterRun.setNewLaunchPosition(currentPos, target);
                hasUpdatedShooterReady = true;
                KLog.d("RoundTrip", String.format("[%s] Shooter position updated successfully",
                    getName() != null ? getName() : "unnamed"));
            }
        }
    }
}
