package com.kalipsorobotics.actions.autoActions.pathActions;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.actions.intake.IntakeFullAction;
import com.kalipsorobotics.actions.shooter.PurePursuitReady;
import com.kalipsorobotics.actions.shooter.ShooterReady;
import com.kalipsorobotics.actions.shooter.ShooterRun;
import com.kalipsorobotics.actions.shooter.ShooterStop;
import com.kalipsorobotics.actions.shooter.pusher.PushBall;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.test.turret.TurretReady;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;

public class RoundTripAction extends KActionSet {
    private final double FINAL_ANGLE_LOCKING_THRESHOLD_DEGREE = 30;
    private PurePursuitAction moveToBall;

    private IntakeFullAction intakeFullAction;

    private ShooterRun shooterRun;
    private Shooter shooter;
    private ShooterReady shooterReady;
    private PushBall pushBall;
    private PurePursuitReady purePursuitReady;
    private ShooterStop shooterStop;
    private TurretAutoAlign turretAutoAlign;
    private TurretReady turretReady;
    private boolean hasUpdatedShooterReady = false;


    private boolean shouldShooterStop = true;


    Point targetPoint;

    public RoundTripAction(OpModeUtilities opModeUtilities, DriveTrain drivetrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake,
                           Point targetPoint, Point launchPoint, double waitForShooterReadyMS, boolean shouldRunIntake, boolean shouldDependOnFlywheel) {
        this.targetPoint = targetPoint;
        this.shooter = shooter;
        this.turretAutoAlign = turretAutoAlign;
        this.shouldShooterStop = shouldShooterStop;

        WaitAction waitUntilShootRun = new WaitAction(waitForShooterReadyMS);
        waitUntilShootRun.setName("waitUntilShootReady");
        this.addAction(waitUntilShootRun);

        // check if this is the correct way to get drivetrain
        PurePursuitAction moveToBalls = new PurePursuitAction(drivetrain);
        moveToBalls.setName("moveToBall");  // FIX: Name the action!
        this.addAction(moveToBalls);
        this.moveToBall = moveToBalls;
        moveToBalls.setFinalAngleLockingThresholdDegree(FINAL_ANGLE_LOCKING_THRESHOLD_DEGREE);
        moveToBalls.setLookAheadRadius(125);  // Increased from default 75 to reduce oscillation during heading changes
        moveToBalls.setFinalSearchRadius(150);
        moveToBalls.setMaxTimeOutMS(8000);

        this.purePursuitReady = new PurePursuitReady(moveToBalls);
        purePursuitReady.setName("purePursuitReady");
        this.addAction(purePursuitReady);

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

        turretReady = new TurretReady(turretAutoAlign);
        turretReady.setName("turretReady");
        turretReady.setDependentActions(moveToBalls);
        this.addAction(turretReady);

        pushBall = new PushBall(stopper, intake, shooter);
        pushBall.setName("shoot");
        if (shouldDependOnFlywheel) {
            pushBall.setDependentActions(purePursuitReady, shooterReady); //removed turretReady
        } else {
            pushBall.setDependentActions(purePursuitReady);
        }
        this.addAction(pushBall);

        shooterStop = new ShooterStop(shooterRun);
        shooterStop.setName("stop");
    }

    public RoundTripAction(OpModeUtilities opModeUtilities, DriveTrain drivetrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake,
                           Point targetPoint, Point launchPos, double waitForShooterReadyMS) {
        this(opModeUtilities, drivetrain, turretAutoAlign, shooter, stopper, intake, targetPoint, launchPos, waitForShooterReadyMS, true, false);
    }

    public RoundTripAction(OpModeUtilities opModeUtilities, DriveTrain drivetrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake,
                           Point targetPoint, Point launchPos, double waitForShooterReadyMS, boolean shouldDependOnFlywheel) {
        this(opModeUtilities, drivetrain, turretAutoAlign, shooter, stopper, intake, targetPoint, launchPos, waitForShooterReadyMS, true, shouldDependOnFlywheel);
    }

    public PurePursuitAction getMoveToBall() {
        return moveToBall;
    }

    public PushBall getPushBall() {
        return pushBall;
    }

    @Override
    protected void beforeUpdate() {
        super.beforeUpdate();

        if (shouldShooterStop && !hasStarted){
            this.addAction(shooterStop);
            shooterStop.setDependentActions(pushBall);
        }
        if (moveToBall.getIsDone()){
            KLog.d("RoundTrip", String.format("[%s] MoveToBall COMPLETED - Stopping intake and updating shooter position",
                getName() != null ? getName() : "unnamed"));

            intakeFullAction.setIsDone(true); //don't need to stop intake because push ball starts intake
            KLog.d("RoundTrip", String.format("[%s] IntakeFullAction stopped by pure pursuit completion",
                getName() != null ? getName() : "unnamed"));

            if (!hasUpdatedShooterReady) {
                Point currentPos = new Position(SharedData.getOdometryWheelIMUPosition()).toPoint();
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

        if (pushBall.getIsDone()) {
            turretReady.setIsDone(true);
            shooterReady.setIsDone(true);
            shooterRun.setIsDone(true);
        }


        KLog.d("RoundTrip", String.format("[%s] Status - MoveToBall: %s, PurePursuitReady: %s, Intake: %s, ShooterReady %s, ShooterRun: %s, PushBall: %s, TurretReady: %s",
                getName() != null ? getName() : "unnamed",
                moveToBall.getIsDone() ? "DONE" : "NOT DONE",
                purePursuitReady.getIsDone() ? "DONE" : "NOT DONE",
                intakeFullAction.getIsDone() ? "DONE" : "NOT DONE",
                shooterReady.getIsDone() ? "DONE" : "NOT DONE",
                shooterRun.getIsDone() ? "DONE" : "NOT DONE",
                pushBall.getIsDone() ? "DONE" : "NOT DONE",
                turretReady.getIsDone() ? "DONE" : "NOT DONE"));
        KLog.d("RoundTrip", String.format("[%s] PP isWithinRange=%b",
                getName() != null ? getName() : "unnamed",
                moveToBall.isWithinRange()));

    }

    public void setShouldShooterStop(boolean shouldShooterStop) {
        this.shouldShooterStop = shouldShooterStop;
    }
}
