package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath.DecisionRoundTripAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.IPurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.HiveTagLocalizer;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeSources;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.SetAutoDelayAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.WaitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeDecisionAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShootDecisionAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterReady;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher.PushBall;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretReady;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.BallVision;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;

@Autonomous
public class RedSmartSoloAuto extends KOpMode {
    final Position ROBOT_START_POS = new Position(973, 3303, 90);
    KActionSet redSmartSoloAuto;
    DriveTrain driveTrain;
    Intake intake;
    FlowerMech flowerMech;
    Shooter shooter;
    Stopper stopper;
    Turret turret;
    final double waitForShooterRunMS = 0.0;

    final double flowerIntakeMS = 2500;
    final Point PARK_POINT = new Point(392, 1371);   // same as RedThreeTipSoloAuto
    final double AUTO_LENGTH_MS = 30000;
    final double FORCE_PARK_MS_LEFT = 3000;   // TODO-check: enough time to drive to PARK_POINT from anywhere?
    final double HIVE_SETTLE_TIMEOUT_MS = 2000;   // TODO-measure: longest we wait for the Hive to stop tipping


    BallVision ballVision = null;
    final BallVision.Detector ballDetector = BallVision.Detector.TFLITE;
    TurretAutoAlign turretAutoAlign;
    @Override
    protected void initializeRobotConfig() {
        super.initializeRobotConfig();
        this.allianceColor = AllianceColor.RED;
        SharedData.setAllianceColor(allianceColor);
    }

    @Override
    protected void initializeRobot() {
        super.initializeRobot();
        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);
        OpModeUtilities.startOdometry(opModeUtilities,odoExecutorService,driveTrain,imuModule,ROBOT_START_POS);
        redSmartSoloAuto = new KActionSet();
        intake = new Intake(opModeUtilities);
        flowerMech = intake.getFlower();
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);
        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);
        turretAutoAlign = new TurretAutoAlign(opModeUtilities,turret);
        startAprilTagDetection(turret);

        SharedData.resetBallInformation();
        ballVision = BallVision.start(hardwareMap,ballDetector,false);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        SetAutoDelayAction setAutoDelayAction = new SetAutoDelayAction(opModeUtilities,gamepad1);
        setAutoDelayAction.setName("setAutoDelayAction");
        while(!setAutoDelayAction.getIsDone()&& opModeInInit()){
            setAutoDelayAction.updateCheckDone();
        }

        WaitAction delayBeforeStart = new WaitAction(setAutoDelayAction.getTimeMS());
        delayBeforeStart.setName("delayBeforeStart");
        redSmartSoloAuto.addAction(delayBeforeStart);

        WaitAction waitUntilShooterRun = new WaitAction(waitForShooterRunMS);
        waitUntilShooterRun.setName("waitUntilShooterRun");
        waitUntilShooterRun.setDependentActions(delayBeforeStart);
        redSmartSoloAuto.addAction(waitUntilShooterRun);

        ShooterRun shooterRunPreload = new ShooterRun(opModeUtilities,shooter, ShooterConfig.TARGET_POINT_1, ROBOT_START_POS.toPoint());   // preload goes into the up Cell with the 3 nectar (Cell A), same as the rigid auto
        shooterRunPreload.setName("shooterRunPreload");
        shooterRunPreload.setDependentActions(waitUntilShooterRun);
        redSmartSoloAuto.addAction(shooterRunPreload);

        ShooterReady shooterReadyPreload = new ShooterReady(shooterRunPreload);
        shooterReadyPreload.setName("shooterReadyPreload");
        shooterReadyPreload.setDependentActions(waitUntilShooterRun);
        redSmartSoloAuto.addAction(shooterReadyPreload);

        TurretReady turretReadyPreload = new TurretReady(turretAutoAlign);
        turretReadyPreload.setName("turretReadyPreload");
        turretReadyPreload.setDependentActions(delayBeforeStart);
        redSmartSoloAuto.addAction(turretReadyPreload);

        PushBall shootPreload = new PushBall(stopper,intake);
        shootPreload.setName("shootPreload");
        shootPreload.setDependentActions(shooterReadyPreload,turretReadyPreload);
        redSmartSoloAuto.addAction(shootPreload);

        ShooterStop shooterStop = new ShooterStop(shooterRunPreload);
        shooterStop.setName("shooterStop");
        shooterStop.setDependentActions(shootPreload);
        redSmartSoloAuto.addAction(shooterStop);
        // TRIPS 1, 2, 3...: built one at a time during the match (see the loop below),
        // so each one decides with what the robot knows right then.
        IntakeSources.resetAll();
        ShootDecisionAction shootDecision = new ShootDecisionAction();
        IntakeDecisionAction intakeDecision = new IntakeDecisionAction(shootDecision);
        DecisionRoundTripAction trip = null;
        int tripNumber = 0;
        boolean nothingLeftToIntake = false;
        ElapsedTime hiveSettleTimer = new ElapsedTime();

        // FORCE PARK: with FORCE_PARK_MS_LEFT left in the match, drop everything and park
        IPurePursuitAction forcePark = new AdaptivePurePursuitAction(driveTrain);
        forcePark.setName("forcePark");
        forcePark.setMaxTimeOutMS(3000); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        forcePark.addPoint(PARK_POINT.getX(), PARK_POINT.getY(), 90); // TODO-check: park heading

        waitForStartPrecomputingPaths();
        ElapsedTime matchTimer = new ElapsedTime();   // starts when the match starts
        boolean forceParking = false;

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            intake.update();
            turretAutoAlign.updateCheckDone(); // the turret only moves while its action is updated

            boolean timeToPark = matchTimer.milliseconds() >= AUTO_LENGTH_MS - FORCE_PARK_MS_LEFT;
            if (timeToPark && !forceParking) {
                forceParking = true;
                stopEverythingButDrive();   // the trip is not updated any more, so nothing turns them back on
                KLog.d("RedSmartSoloAuto", "force park at " + matchTimer.milliseconds() + " ms");
            }

            boolean preloadRunning = !redSmartSoloAuto.getIsDone();
            boolean tripRunning = (trip != null && !trip.getIsDone());

            if (forceParking) {
                forcePark.updateCheckDone();   // no more preload or trips

            } else if (preloadRunning) {
                redSmartSoloAuto.updateCheckDone();
                hiveSettleTimer.reset();   // start the settle wait from the last shot

            } else if (tripRunning) {
                trip.updateCheckDone();
                hiveSettleTimer.reset();

            } else if (!nothingLeftToIntake) {
                // Between trips. Let the Hive finish tipping before we look at it.
                boolean hiveStillMoving = SharedData.isHiveTipping() && hiveSettleTimer.milliseconds() < HIVE_SETTLE_TIMEOUT_MS;
                if (!hiveStillMoving) {
                    if (trip != null) {
                        updatePlacesAfterTrip(intakeDecision, shootDecision);
                    }

                    tripNumber++;
                    trip = new DecisionRoundTripAction(opModeUtilities, driveTrain, shooter, stopper, intake, flowerMech,
                            turretAutoAlign, shootDecision, intakeDecision, flowerIntakeMS);
                    trip.setName("trip" + tripNumber);

                    if (intakeDecision.getChosenSource() == null) {
                        nothingLeftToIntake = true;
                        forceParking = true;   // nothing left to intake: park now, like the rigid auto
                        stopEverythingButDrive();
                        KLog.d("RedSmartSoloAuto", "nothing left to intake, parking at " + matchTimer.milliseconds() + " ms");
                    }
                }
            }

            KLog.d("Odometry", () -> "Position: " + SharedData.getOdometryWheelIMUPosition());
        }
        shooter.stop();
        ballVision.close();
        cleanupRobot();
    }

    /** Shooter, intake and turret off: parking only needs the drivetrain. */
    private void stopEverythingButDrive() {
        shooter.stop();
        intake.stop();
        turretAutoAlign.stop();
        KLog.d("RedSmartSoloAuto", "stopped shooter, intake and turret before parking");
    }

    /**
     * Called after a trip finished and the Hive settled.
     * 1. The place we just intook from is now emptied.
     * 2. If the Cell we shot at is no longer up, it tipped: its balls fell into the tip zone on the same side.
     *    If it is still up, it did NOT tip, so the next trip shoots it again (ShootDecisionAction aims at the up Cell).
     */
    private void updatePlacesAfterTrip(IntakeDecisionAction intakeDecision, ShootDecisionAction shootDecision) {
        IntakeSources lastPlace = intakeDecision.getChosenSource();
        if (lastPlace != null) {
            lastPlace.subtractIntakeValue();
        }

        HiveTagLocalizer.Cell cellWeShotAt = shootDecision.getAimedCell();
        HiveTagLocalizer.Cell cellUpNow = SharedData.getHiveUpCell();
        boolean itTipped = (cellUpNow != null && cellUpNow != cellWeShotAt);

        if (itTipped) {
            if (cellWeShotAt == HiveTagLocalizer.Cell.A) {
                IntakeSources.TIP_ZONE_1.addIntakeValue();   // Cell A side
            } else {
                IntakeSources.TIP_ZONE_2.addIntakeValue();   // Cell B side
            }
        }
        KLog.d("RedSmartSoloAuto", "after trip: emptied " + lastPlace + ", shot at Cell " + cellWeShotAt
                + ", up now " + cellUpNow + ", tipped=" + itTipped);
    }
}
