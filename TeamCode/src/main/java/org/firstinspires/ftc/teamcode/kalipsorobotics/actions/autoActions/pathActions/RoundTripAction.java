package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.WaitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeFullAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitReady;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterReady;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher.PushBall;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretReadyAuto;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

public class RoundTripAction extends KActionSet {
    private final double FINAL_ANGLE_LOCKING_THRESHOLD_DEGREE = 30;
    private final PurePursuitAction moveToBall;

    private final IntakeFullAction intakeFullAction;

    private final ShooterRun shooterRun;
    private final Shooter shooter;



    private final ShooterReady shooterReady;
    private final PushBall pushBall;
    private final PurePursuitReady purePursuitReadyShooting;

    private final PurePursuitReady purePursuitReadyIntakeStop;
    private final ShooterStop shooterStop;
    private final TurretAutoAlign turretAutoAlign;
    private final TurretReadyAuto turretReadyAuto;
    private boolean hasUpdatedShooterReady = false;
    private boolean shouldShooterStop = true;


    Point targetPoint;

    public RoundTripAction(OpModeUtilities opModeUtilities, DriveTrain drivetrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake,
                           Point targetPoint, Point launchPoint, double waitForShooterReadyMS, boolean shouldRunIntake, boolean shouldDependOnFlywheel) {
        this.targetPoint = targetPoint;
        this.shooter = shooter;
        this.turretAutoAlign = turretAutoAlign;

        WaitAction waitUntilShootRun = new WaitAction(waitForShooterReadyMS);
        waitUntilShootRun.setName("waitUntilShootReady");
        this.addAction(waitUntilShootRun);

        PurePursuitAction moveToBalls = new PurePursuitAction(drivetrain);
        moveToBalls.setName("moveToBall");
        this.addAction(moveToBalls);
        this.moveToBall = moveToBalls;
        moveToBalls.setFinalAngleLockingThresholdDeg(FINAL_ANGLE_LOCKING_THRESHOLD_DEGREE);
        moveToBalls.setLookAheadRadius(125);  // Increased from default 75 to reduce oscillation during heading changes
        moveToBalls.setFinalSearchRadiusMM(150);
        moveToBalls.setMaxTimeOutMS(8000);

        purePursuitReadyIntakeStop = new PurePursuitReady(moveToBalls, 600);
        purePursuitReadyIntakeStop.setName("purePursuitReadyIntakeStop");
        this.addAction(purePursuitReadyIntakeStop);


        intakeFullAction = new IntakeFullAction(stopper, intake, IntakeConfig.intakeBallTimeMS, IntakeConfig.intakePower);
        intakeFullAction.setName("intakeFullAction");
        this.addAction(intakeFullAction);

        if (!shouldRunIntake) {
            intakeFullAction.setIsDone(true);
        }

        IntakeStop intakeStop = new IntakeStop(intakeFullAction);
        intakeStop.setName("intakeStop");
        intakeStop.setDependentActions(purePursuitReadyIntakeStop);
        this.addAction(intakeStop);

        this.purePursuitReadyShooting = new PurePursuitReady(moveToBalls, moveToBall.getLastSearchRadiusMM());
        purePursuitReadyShooting.setName("purePursuitReadyShooting");
        this.addAction(purePursuitReadyShooting);

        //warm - shorter
        shooterRun = new ShooterRun(opModeUtilities, shooter, targetPoint, launchPoint);
        shooterRun.setName("shooterRun");
        shooterRun.setDependentActions(waitUntilShootRun);
        this.addAction(shooterRun);

        shooterReady = new ShooterReady(shooterRun);
        shooterReady.setName("shooterReady");
        shooterReady.setDependentActions(moveToBalls);
        this.addAction(shooterReady);

        turretReadyAuto = new TurretReadyAuto(turretAutoAlign);
        turretReadyAuto.setName("turretReady");
        turretReadyAuto.setDependentActions(moveToBalls);
        this.addAction(turretReadyAuto);

        TurretStop turretStop = new TurretStop(turretAutoAlign);
        turretStop.setName("turretStop");
        turretStop.setDependentActions(turretReadyAuto);
        this.addAction(turretStop);

        pushBall = new PushBall(stopper, intake);
        pushBall.setName("shoot");
        if (shouldDependOnFlywheel) {
            pushBall.setDependentActions(purePursuitReadyShooting, shooterReady, turretReadyAuto); //removed turretReady
        } else {
            pushBall.setDependentActions(purePursuitReadyShooting, turretReadyAuto);
        }
//        pushBall.getRunUntilFullSpeed().setFullSpeedDurationMs(150);
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
            hasStarted = true;
            this.addAction(shooterStop);
            shooterStop.setDependentActions(pushBall);
        }
        if (moveToBall.getIsDone()){
            KLog.d("RoundTrip", () -> String.format("[%s] MoveToBall COMPLETED - Stopping intake and updating shooter position",
                getName() != null ? getName() : "unnamed"));

            if (!hasUpdatedShooterReady) {
                Point currentPos = new Position(SharedData.getOdometryWheelIMUPosition()).toPoint();
                KLog.d("RoundTrip", () -> String.format("[%s] Updating shooter position - Current: (%.1f, %.1f), Target: (%.1f, %.1f)",
                    getName() != null ? getName() : "unnamed",
                    currentPos.getX(), currentPos.getY(),
                    targetPoint.getX(), targetPoint.getY()));
//                shooterRun.setNewLaunchPosition(currentPos, target);
                hasUpdatedShooterReady = true;
                KLog.d("RoundTrip", () -> String.format("[%s] Shooter position updated successfully",
                    getName() != null ? getName() : "unnamed"));
            }
        }


        KLog.d("RoundTrip", () -> String.format("[%s] Status - MoveToBall: %s, PurePursuitReady: %s, Intake: %s, ShooterReady %s, ShooterRun: %s, PushBall: %s, TurretReady: %s",
                getName() != null ? getName() : "unnamed",
                moveToBall.getIsDone() ? "DONE" : "NOT DONE",
                purePursuitReadyShooting.getIsDone() ? "DONE" : "NOT DONE",
                intakeFullAction.getIsDone() ? "DONE" : "NOT DONE",
                shooterReady.getIsDone() ? "DONE" : "NOT DONE",
                shooterRun.getIsDone() ? "DONE" : "NOT DONE",
                pushBall.getIsDone() ? "DONE" : "NOT DONE",
                turretReadyAuto.getIsDone() ? "DONE" : "NOT DONE"));
    }

    @Override
    public void afterUpdate() {
        if (pushBall.getIsDone()) {
            turretReadyAuto.setIsDone(true);
            shooterReady.setIsDone(true);
            shooterRun.setIsDone(true);
            intakeFullAction.setIsDone(true);
        }

        if (moveToBall.getIsDone()) {
            intakeFullAction.setIsDone(true); //don't need to stop intake because push ball starts intake

            KLog.d("RoundTrip", () -> String.format("[%s] IntakeFullAction stopped by pure pursuit completion",
                    getName() != null ? getName() : "unnamed"));
        }
    }

    public void setShouldShooterStop(boolean shouldShooterStop) {
        this.shouldShooterStop = shouldShooterStop;
    }

    public ShooterReady getShooterReady() {
        return shooterReady;
    }
    public IntakeFullAction getIntakeFullAction() {
        return intakeFullAction;
    }


    public PurePursuitReady getPurePursuitReadyShooting() {
        return purePursuitReadyShooting;
    }

    public PurePursuitReady getPurePursuitReadyIntakeStop() {
        return purePursuitReadyIntakeStop;
    }
}
