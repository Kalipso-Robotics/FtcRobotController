package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.WaitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeFullAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterReady;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher.PushBall;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretReady;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitReady;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.IPurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;

/**
 * One auto trip. Two shapes:
 *
 * 1. GATED TRIP (master constructor below, rebuilt from last year's decode/RoundTripAction): drive moveToBall (to the balls, then
 *    on to the launch spot; add the points) with intakeSide running; the intake stops INTAKE_STOP_BEFORE_END_MM before the end
 *    (purePursuitReadyIntakeStop); ShooterRun warms up during the drive; PushBall waits for the drive (purePursuitReadyShooting),
 *    ShooterReady (if shouldDependOnFlywheel) and TurretReady; then ShooterStop (setShouldShooterStop(false) keeps it spinning).
 *    Auto: {@code RoundTripAction trip = new RoundTripAction(opModeUtilities, driveTrain, aim, shooter, stopper, intake,
 *          ShooterConfig.TARGET_POINT_1, FieldConfig.aLaunchPoint, 0, intake.getSide(BeamBreakPosition.IntakeSide.A));
 *          trip.getMoveToBall().addPoint(...);}
 *
 * 2. SHOOT-FIRST TRIP (the two older BIOBUZZ constructors, kept unchanged for RedTippingAuto): moveToShoot, shoot, then
 *    moveToBalls with both intakes on. getMoveToShoot()/getMoveToBalls() belong to this shape only.
 */
public class RoundTripAction extends KActionSet {
    // Default path tuning for the gated trip (last year's values).
    private static final double FINAL_ANGLE_LOCKING_THRESHOLD_DEGREE = 30;
    private static final double LOOK_AHEAD_RADIUS_MM = 125;       // up from the default 75: less oscillation in heading changes
    private static final double FINAL_SEARCH_RADIUS_MM = 150;
    private static final double MAX_TIMEOUT_MS = 8000;
    private static final double INTAKE_STOP_BEFORE_END_MM = 900;

    // ---- shoot-first trip (RedTippingAuto)
    IPurePursuitAction moveToShoot;
    IntakeFullAction intakeBalls;
    IPurePursuitAction moveToBalls;

    // ---- gated trip
    private IPurePursuitAction moveToBall;
    private IntakeFullAction intakeFullAction;     // null when the trip runs no intake
    private ShooterRun shooterRun;
    private ShooterReady shooterReady;
    private PushBall pushBall;
    private PurePursuitReady purePursuitReadyShooting;
    private PurePursuitReady purePursuitReadyIntakeStop;
    private ShooterStop shooterStop;
    private TurretReady turretReadyAuto;
    private boolean shouldShooterStop = true;
    private boolean shooterStopAdded = false;
    private boolean wasShootFired = false;
    private Point launchPoint;
    private TurretAutoAlign aim;                   // the turret only moves while this is updated: the trip updates it until the shot

    /**
     * Gated trip (see the class comment). Pattern source: actions/autoActions/pathActions/decode/RoundTripAction.
     *
     * @param waitForShooterReadyMS delay before the flywheel starts warming up
     * @param intakeSide            the side that picks up on the way, or null for no intake (e.g. the preload shot)
     */
    public RoundTripAction(OpModeUtilities opModeUtilities, DriveTrain drivetrain, TurretAutoAlign turretAutoAlign, Shooter shooter,
                           Stopper stopper, Intake intake, Point targetPoint, Point launchPoint, double waitForShooterReadyMS,
                           IntakeSide intakeSide, boolean shouldDependOnFlywheel, boolean useAdaptivePP) {
        this.launchPoint = launchPoint;
        this.aim = turretAutoAlign;

        WaitAction waitUntilShootRun = new WaitAction(waitForShooterReadyMS);
        waitUntilShootRun.setName("waitUntilShootRun");
        this.addAction(waitUntilShootRun);

        moveToBall = useAdaptivePP ? new AdaptivePurePursuitAction(drivetrain) : new PurePursuitAction(drivetrain);
        moveToBall.setName("moveToBall");
        moveToBall.setFinalAngleLockingThresholdDeg(FINAL_ANGLE_LOCKING_THRESHOLD_DEGREE);
        moveToBall.setLookAheadRadius(LOOK_AHEAD_RADIUS_MM);
        moveToBall.setFinalSearchRadiusMM(FINAL_SEARCH_RADIUS_MM);
        moveToBall.setMaxTimeOutMS(MAX_TIMEOUT_MS);
        this.addAction(moveToBall);

        purePursuitReadyIntakeStop = new PurePursuitReady(moveToBall, INTAKE_STOP_BEFORE_END_MM);
        purePursuitReadyIntakeStop.setName("purePursuitReadyIntakeStop");
        this.addAction(purePursuitReadyIntakeStop);

        if (intakeSide != null) {
            intakeFullAction = new IntakeFullAction(stopper, intakeSide, IntakeConfig.intakeBallTimeMS, IntakeConfig.intakePower);
            intakeFullAction.setName("intakeFullAction");
            this.addAction(intakeFullAction);

            IntakeStop intakeStop = new IntakeStop(intakeFullAction);
            intakeStop.setName("intakeStop");
            intakeStop.setDependentActions(purePursuitReadyIntakeStop);
            this.addAction(intakeStop);
        }

        purePursuitReadyShooting = new PurePursuitReady(moveToBall, moveToBall.getLastSearchRadiusMM());
        purePursuitReadyShooting.setName("purePursuitReadyShooting");
        this.addAction(purePursuitReadyShooting);

        shooterRun = new ShooterRun(opModeUtilities, shooter, targetPoint, launchPoint); // warms up during the drive
        shooterRun.setName("shooterRun");
        shooterRun.setDependentActions(waitUntilShootRun);
        this.addAction(shooterRun);

        shooterReady = new ShooterReady(shooterRun);
        shooterReady.setName("shooterReady");
        shooterReady.setDependentActions(moveToBall);
        this.addAction(shooterReady);

        turretReadyAuto = new TurretReady(turretAutoAlign);
        turretReadyAuto.setName("turretReady");
        turretReadyAuto.setDependentActions(moveToBall);
        this.addAction(turretReadyAuto);

        pushBall = new PushBall(stopper, intake);
        pushBall.setName("shoot");
        if (shouldDependOnFlywheel) {
            pushBall.setDependentActions(purePursuitReadyShooting, shooterReady, turretReadyAuto);
        } else {
            pushBall.setDependentActions(purePursuitReadyShooting, turretReadyAuto);
        }
        this.addAction(pushBall);

        shooterStop = new ShooterStop(shooterRun);
        shooterStop.setName("shooterStop");
        shooterStop.setDependentActions(pushBall);
    }

    /** Gated trip, waits for the flywheel, adaptive pure pursuit. */
    public RoundTripAction(OpModeUtilities opModeUtilities, DriveTrain drivetrain, TurretAutoAlign turretAutoAlign, Shooter shooter,
                           Stopper stopper, Intake intake, Point targetPoint, Point launchPoint, double waitForShooterReadyMS,
                           IntakeSide intakeSide) {
        this(opModeUtilities, drivetrain, turretAutoAlign, shooter, stopper, intake, targetPoint, launchPoint, waitForShooterReadyMS,
                intakeSide, true, true);
    }

    /** Gated trip, adaptive pure pursuit. */
    public RoundTripAction(OpModeUtilities opModeUtilities, DriveTrain drivetrain, TurretAutoAlign turretAutoAlign, Shooter shooter,
                           Stopper stopper, Intake intake, Point targetPoint, Point launchPoint, double waitForShooterReadyMS,
                           IntakeSide intakeSide, boolean shouldDependOnFlywheel) {
        this(opModeUtilities, drivetrain, turretAutoAlign, shooter, stopper, intake, targetPoint, launchPoint, waitForShooterReadyMS,
                intakeSide, shouldDependOnFlywheel, true);
    }


    public RoundTripAction(OpModeUtilities opModeUtilities, DriveTrain drivetrain, /* TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake, */ AllianceColor allianceColor, boolean useAdaptivePP) {
        //turretAutoAlign.setToleranceDeg(1.5);

        if (!useAdaptivePP) {
            moveToShoot = new PurePursuitAction(drivetrain);
        } else {
            moveToShoot = new AdaptivePurePursuitAction(drivetrain);
        }
        moveToShoot.setName("moveToShoot");
        this.addAction(moveToShoot);

        // shoot stuff

        if (!useAdaptivePP) {
            moveToBalls = new PurePursuitAction(drivetrain);
        } else {
            AdaptivePurePursuitAction adaptiveMoveToBalls = new AdaptivePurePursuitAction(drivetrain);
            adaptiveMoveToBalls.setPlanStartFrom(moveToShoot);
            moveToBalls = adaptiveMoveToBalls;
        }
        moveToBalls.setName("moveToBalls");
        moveToBalls.setDependentActions(moveToShoot);
        this.addAction(moveToBalls);

//        intakeBalls = new IntakeFullAction(stopper, intake, intakeTimeMS, 1);
//        intakeBalls.setName("intakeBalls");
//        intakeBalls.setDependentActions(moveToBalls);
//        this.addAction(intakeBalls);
    }

    /**
     * Same trip as above, with the shooting and intake switched on: drive to the shoot point, shoot at the target Cell, then drive
     * to the balls with the intake running (a Garden or a Flower is picked up by driving into it). The intake stops when the
     * drive to the balls ends.
     */
    public RoundTripAction(OpModeUtilities opModeUtilities, DriveTrain drivetrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake,
                           Point targetPoint, Point launchPoint, AllianceColor allianceColor, boolean useAdaptivePP) {
        this(opModeUtilities, drivetrain, allianceColor, useAdaptivePP);

        // the flywheel warms up while the robot drives to the shoot point
        ShooterRun shooterRun = new ShooterRun(opModeUtilities, shooter, targetPoint, launchPoint);
        shooterRun.setName("shooterRun");
        this.addAction(shooterRun);

        ShooterReady shooterReady = new ShooterReady(shooterRun);
        shooterReady.setName("shooterReady");
        shooterReady.setDependentActions(moveToShoot);
        this.addAction(shooterReady);

        TurretReady turretReady = new TurretReady(turretAutoAlign);
        turretReady.setName("turretReady");
        turretReady.setDependentActions(moveToShoot);
        this.addAction(turretReady);

        PushBall pushBall = new PushBall(stopper, intake);
        pushBall.setName("shoot");
        pushBall.setDependentActions(moveToShoot, turretReady);
        this.addAction(pushBall);

        ShooterStop shooterStop = new ShooterStop(shooterRun);
        shooterStop.setName("shooterStop");
        shooterStop.setDependentActions(pushBall);
        this.addAction(shooterStop);

        // after the shot: drive to the balls and intake on the way
        moveToBalls.setDependentActions(moveToShoot, pushBall);

        intakeBalls = new IntakeFullAction(stopper, intake, 10000, IntakeConfig.intakePower);
        intakeBalls.setName("intakeBalls");
        intakeBalls.setDependentActions(pushBall);
        this.addAction(intakeBalls);

        IntakeStop intakeStop = new IntakeStop(intakeBalls);
        intakeStop.setName("intakeStop");
        intakeStop.setDependentActions(moveToBalls);
        this.addAction(intakeStop);
    }

    /** Shoot-first trip only (null on a gated trip). */
    public IPurePursuitAction getMoveToShoot() {return moveToShoot;}
    /** Shoot-first trip only (null on a gated trip). */
    public IPurePursuitAction getMoveToBalls() {return moveToBalls;}

    // ---- gated trip getters (null on a shoot-first trip), same as last year
    public IPurePursuitAction getMoveToBall() {return moveToBall;}
    public ShooterReady getShooterReady() {return shooterReady;}
    public PushBall getPushBall() {return pushBall;}
    public IntakeFullAction getIntakeFullAction() {return intakeFullAction;}
    public PurePursuitReady getPurePursuitReadyShooting() {return purePursuitReadyShooting;}
    public PurePursuitReady getPurePursuitReadyIntakeStop() {return purePursuitReadyIntakeStop;}

    /** Gated trip: false keeps the flywheel spinning after the shot (the next trip warms up faster). Set before the trip starts. */
    public void setShouldShooterStop(boolean shouldShooterStop) {
        this.shouldShooterStop = shouldShooterStop;
    }

    @Override
    protected void beforeUpdate() {
        super.beforeUpdate();
        if (pushBall != null && shouldShooterStop && !shooterStopAdded) {
            shooterStopAdded = true;
            this.addAction(shooterStop);
        }
        if (pushBall != null && !pushBall.getIsDone()) {
            aim.updateCheckDone(); // TurretAutoAlign never finishes, so it is not in the set; it aims until the shot
        }
    }

    @Override
    public void afterUpdate() {
        if (pushBall == null) {
            return; // shoot-first trip: nothing to tidy
        }
        if (pushBall.getIsDone()) {
            turretReadyAuto.setIsDone(true);
            shooterReady.setIsDone(true);
            shooterRun.setIsDone(true);
            if (intakeFullAction != null) {
                intakeFullAction.setIsDone(true);
            }
        }
        if (!wasShootFired && pushBall.getIsDone()) {
            wasShootFired = true;
            Point robot = SharedData.getOdometryWheelIMUPosition().toPoint();
            double fromLaunch = launchPoint != null ? robot.distanceTo(launchPoint) : -1;
            KLog.d("Auto_RoundTrip", () -> String.format("name=%s event=shotFired x=%.0f y=%.0f distFromLaunchMm=%.0f",
                    getName(), robot.getX(), robot.getY(), fromLaunch));
        }
        if (moveToBall.getIsDone() && intakeFullAction != null) {
            intakeFullAction.setIsDone(true); // PushBall runs the intake itself for the feed
        }
    }

}
