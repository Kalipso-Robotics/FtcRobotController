package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.SetAutoDelayAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.WaitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath.VisionPickupAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.Flower;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.FullFlowerAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeFullAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterReady;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher.PushBall;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretReady;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.IPurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.BallVision;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;

@Autonomous
public class ThreeTipSoloAuto extends KOpMode {
    KActionSet redThreeTipSoloAuto;
    public DriveTrain driveTrain;
    Shooter shooter = null;
    Intake intake = null;
    Stopper stopper = null;
    Turret turret = null;
    FlowerMech flowerMech = null;
    TurretAutoAlign turretAutoAlign = null;
    BallVision ballVision = null;

    final double waitForShooterRunMS = 0.0;
    final double flowerIntakeMS = 2500;            // TODO-measure: longest the intake runs on a flower trip (it also stops once backed out)
    final double visionPickupTimeoutMS = 6000;     // TODO-measure: trip 4 gives up picking up after this, to leave time to shoot and park
    final BallVision.Detector ballDetector = BallVision.Detector.TFLITE; // or COLOR_BLOB (tune its HSV first)

    final Position ROBOT_START_POS = new Position(996, 3360, Math.toRadians(-90)); // faces -y (into the field); theta is RADIANS. -90 confirmed from 2026-11-26 TeleOp drive log
    final Point PARK_POINT = new Point(392, 1371);
    final double AUTO_LENGTH_MS = 30000;
    static final boolean USE_LIMELIGHT = false; // nothing in this auto reads the Limelight yet; true = start it anyway
    final double FORCE_PARK_MS_LEFT = 3000;   // TODO-check: enough time to drive to PARK_POINT from anywhere?


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
        OpModeUtilities.startOdometry(opModeUtilities, odoExecutorService, driveTrain, imuModule, ROBOT_START_POS);
        redThreeTipSoloAuto = new KActionSet();
        intake = new Intake(opModeUtilities);
        flowerMech = intake.getFlower();
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);
        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);
        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret); // follows SharedData.getTargetPoint() (Cell on the robot's end); each ShooterRun names its own Cell
        if (USE_LIMELIGHT) startAprilTagDetection(turret);
        // No BeamBreakCountAction: the beam breaks are not on the robot yet (V2), and building it needs their sensors in the
        // hardware map. Trip 4 knows the intake is full from the intake motor current instead (VisionPickupAction).

        // Camera for trip 4: every ball it sees goes into SharedData.getBallInformation() (field mm).
        SharedData.resetBallInformation();
        ballVision = BallVision.start(hardwareMap, ballDetector, false);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        // TRIP 0
        SetAutoDelayAction setAutoDelayAction = new SetAutoDelayAction(opModeUtilities, gamepad1);
        setAutoDelayAction.setName("setAutoDelayAction");
        while (!setAutoDelayAction.getIsDone() && opModeInInit()) {
            setAutoDelayAction.updateCheckDone();
            initLoop();
        }

        WaitAction delayBeforeStart = new WaitAction(setAutoDelayAction.getTimeMS());
        delayBeforeStart.setName("delayBeforeStart");
        redThreeTipSoloAuto.addAction(delayBeforeStart);

        WaitAction trip0WaitUntilShooterRun = new WaitAction(waitForShooterRunMS);
        trip0WaitUntilShooterRun.setName("trip0WaitUntilShooterRun");
        trip0WaitUntilShooterRun.setDependentActions(delayBeforeStart);
        redThreeTipSoloAuto.addAction(trip0WaitUntilShooterRun);

        // Preload goes into the up Cell, the one with the 3 nectar (Cell A). From the start pose the turret also picks Cell A (SharedData.getTargetPoint).
        ShooterRun trip0ShooterRun = new ShooterRun(opModeUtilities, shooter, ShooterConfig.TARGET_POINT_1, ROBOT_START_POS.toPoint());
        trip0ShooterRun.setName("trip0ShooterRun");
        trip0ShooterRun.setDependentActions(trip0WaitUntilShooterRun);
        redThreeTipSoloAuto.addAction(trip0ShooterRun);

        ShooterReady trip0ShooterReady = new ShooterReady(trip0ShooterRun);
        trip0ShooterReady.setName("trip0ShooterReady");
        trip0ShooterReady.setDependentActions(trip0WaitUntilShooterRun);
        redThreeTipSoloAuto.addAction(trip0ShooterReady);

        TurretReady trip0TurretReady = new TurretReady(turretAutoAlign);
        trip0TurretReady.setName("trip0TurretReady");
        trip0TurretReady.setDependentActions(delayBeforeStart);
        redThreeTipSoloAuto.addAction(trip0TurretReady);

        PushBall trip0Shoot = new PushBall(stopper, intake);
        trip0Shoot.setName("trip0Shoot");
        trip0Shoot.setDependentActions(trip0ShooterReady, trip0TurretReady);
        redThreeTipSoloAuto.addAction(trip0Shoot);

        ShooterStop trip0ShooterStop = new ShooterStop(trip0ShooterRun);
        trip0ShooterStop.setName("trip0ShooterStop");
        trip0ShooterStop.setDependentActions(trip0Shoot);
        redThreeTipSoloAuto.addAction(trip0ShooterStop);
        //END OF TRIP 0

        //TRIP 1
        FullFlowerAction trip1FlowerIntake = new FullFlowerAction(stopper, intake, flowerIntakeMS, IntakeConfig.intakePower, flowerMech, driveTrain, Flower.FLOWER_2);
        trip1FlowerIntake.setName("trip1FlowerIntake");
        trip1FlowerIntake.setDependentActions(trip0Shoot);
        trip1FlowerIntake.getFlowerLower().setIsDone(true);
        redThreeTipSoloAuto.addAction(trip1FlowerIntake);

        IPurePursuitAction trip1GoToShoot = new AdaptivePurePursuitAction(driveTrain);
        trip1GoToShoot.setMaxTimeOutMS(4000); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        trip1GoToShoot.setName("trip1GoToShoot");
        trip1GoToShoot.addPoint(1052, 543, 289.1);
        trip1GoToShoot.setDependentActions(trip1FlowerIntake);
        redThreeTipSoloAuto.addAction(trip1GoToShoot);

        ShooterRun trip1ShooterRun = new ShooterRun(opModeUtilities, shooter, ShooterConfig.TARGET_POINT_2, new Point(1052, 543));
        trip1ShooterRun.setName("trip1ShooterRun");
        trip1ShooterRun.setDependentActions(trip1GoToShoot);
        redThreeTipSoloAuto.addAction(trip1ShooterRun);

        ShooterReady trip1ShooterReady = new ShooterReady(trip1ShooterRun);
        trip1ShooterReady.setName("trip1ShooterReady");
        trip1ShooterReady.setDependentActions(trip1GoToShoot);
        redThreeTipSoloAuto.addAction(trip1ShooterReady);

        TurretReady trip1TurretReady = new TurretReady(turretAutoAlign);
        trip1TurretReady.setName("trip1TurretReady");
        trip1TurretReady.setDependentActions(trip1GoToShoot);
        redThreeTipSoloAuto.addAction(trip1TurretReady);

        PushBall trip1Shoot = new PushBall(stopper, intake);
        trip1Shoot.setName("trip1Shoot");
        trip1Shoot.setDependentActions(trip1ShooterReady, trip1TurretReady);
        redThreeTipSoloAuto.addAction(trip1Shoot);

        ShooterStop trip1ShooterStop = new ShooterStop(trip1ShooterRun);
        trip1ShooterStop.setName("trip1ShooterStop");
        trip1ShooterStop.setDependentActions(trip1Shoot);
        redThreeTipSoloAuto.addAction(trip1ShooterStop);
        //END OF TRIP 1

        //TRIP 2
        FullFlowerAction trip2FlowerIntake = new FullFlowerAction(stopper, intake, flowerIntakeMS, IntakeConfig.intakePower, flowerMech, driveTrain, Flower.FLOWER_1);
        trip2FlowerIntake.setName("trip2FlowerIntake");
        trip2FlowerIntake.setDependentActions(trip1Shoot);
        redThreeTipSoloAuto.addAction(trip2FlowerIntake);

        // TODO-check: added so the robot is at (1037, 397) for this shot, same as trip 1
        IPurePursuitAction trip2GoToShoot = new AdaptivePurePursuitAction(driveTrain);
        trip2GoToShoot.setMaxTimeOutMS(1500); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        trip2GoToShoot.setName("trip2GoToShoot");
        trip2GoToShoot.addPoint(1052, 543, 289.1);
        trip2GoToShoot.setDependentActions(trip2FlowerIntake);
        redThreeTipSoloAuto.addAction(trip2GoToShoot);

        ShooterRun trip2ShooterRun = new ShooterRun(opModeUtilities, shooter, ShooterConfig.TARGET_POINT_2, new Point(1052, 543));
        trip2ShooterRun.setName("trip2ShooterRun");
        trip2ShooterRun.setDependentActions(trip2GoToShoot);
        redThreeTipSoloAuto.addAction(trip2ShooterRun);

        ShooterReady trip2ShooterReady = new ShooterReady(trip2ShooterRun);
        trip2ShooterReady.setName("trip2ShooterReady");
        trip2ShooterReady.setDependentActions(trip2GoToShoot);
        redThreeTipSoloAuto.addAction(trip2ShooterReady);

        TurretReady trip2TurretReady = new TurretReady(turretAutoAlign);
        trip2TurretReady.setName("trip2TurretReady");
        trip2TurretReady.setDependentActions(trip2GoToShoot);
        redThreeTipSoloAuto.addAction(trip2TurretReady);

        PushBall trip2Shoot = new PushBall(stopper, intake);
        trip2Shoot.setName("trip2Shoot");
        trip2Shoot.setDependentActions(trip2ShooterReady, trip2TurretReady);
        redThreeTipSoloAuto.addAction(trip2Shoot);

        ShooterStop trip2ShooterStop = new ShooterStop(trip2ShooterRun);
        trip2ShooterStop.setName("trip2ShooterStop");
        trip2ShooterStop.setDependentActions(trip2Shoot);
        redThreeTipSoloAuto.addAction(trip2ShooterStop);
        //END OF TRIP 2

        //TRIP 3

        IPurePursuitAction trip3GoToGarden = new AdaptivePurePursuitAction(driveTrain);
        trip3GoToGarden.setMaxTimeOutMS(4500); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        trip3GoToGarden.setName("trip3GoToGarden");
        trip3GoToGarden.addPoint(800, 543, 102.6);
        trip3GoToGarden.addPoint(300, 2949, 102.6);
        trip3GoToGarden.setDependentActions(trip2ShooterStop);
        redThreeTipSoloAuto.addAction(trip3GoToGarden);

        IntakeFullAction trip3IntakeFullAction = new IntakeFullAction(stopper, intake, IntakeConfig.intakeBallTimeMS, IntakeConfig.intakePower);
        trip3IntakeFullAction.setName("trip3IntakeFullAction");
        trip3IntakeFullAction.setDependentActions(trip3GoToGarden);
        redThreeTipSoloAuto.addAction(trip3IntakeFullAction);

        IPurePursuitAction trip3IntakeSweepGarden = new AdaptivePurePursuitAction(driveTrain);
        trip3IntakeSweepGarden.setMaxTimeOutMS(3000); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        trip3IntakeSweepGarden.setName("trip3IntakeGarden");
        trip3IntakeSweepGarden.addPoint(195, 3309, 90.0);
        trip3IntakeSweepGarden.addPoint(240, 3200, 95.0);
        trip3IntakeSweepGarden.addPoint(195, 3309, 90.0);
        trip3IntakeSweepGarden.setDependentActions(trip3GoToGarden);
        redThreeTipSoloAuto.addAction(trip3IntakeSweepGarden);

        // Sweep done: stop the intake now, so its timer does not keep it running into the shot (same as FullFlowerAction).
        IntakeStop trip3StopIntake = new IntakeStop(trip3IntakeFullAction);
        trip3StopIntake.setName("trip3StopIntake");
        trip3StopIntake.setDependentActions(trip3IntakeSweepGarden);
        redThreeTipSoloAuto.addAction(trip3StopIntake);

        IPurePursuitAction trip3GoToShoot = new AdaptivePurePursuitAction(driveTrain);
        trip3GoToShoot.setMaxTimeOutMS(1500); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        trip3GoToShoot.setName("trip3GoToShoot");
        trip3GoToShoot.addPoint(941, 3089, 155.9);
        trip3GoToShoot.setDependentActions(trip3IntakeSweepGarden);
        redThreeTipSoloAuto.addAction(trip3GoToShoot);

        ShooterRun trip3GardenShooterRun = new ShooterRun(opModeUtilities, shooter, ShooterConfig.TARGET_POINT_1, new Point(941, 3089));
        trip3GardenShooterRun.setName("trip3GardenShooterRun");
        trip3GardenShooterRun.setDependentActions(trip3GoToShoot);
        redThreeTipSoloAuto.addAction(trip3GardenShooterRun);

        ShooterReady trip3GardenShooterReady = new ShooterReady(trip3GardenShooterRun);
        trip3GardenShooterReady.setName("trip3GardenShooterReady");
        trip3GardenShooterReady.setDependentActions(trip3GoToShoot);
        redThreeTipSoloAuto.addAction(trip3GardenShooterReady);

        TurretReady trip3GardenTurretReady = new TurretReady(turretAutoAlign);
        trip3GardenTurretReady.setName("trip3GardenTurretReady");
        trip3GardenTurretReady.setDependentActions(trip3GoToShoot);
        redThreeTipSoloAuto.addAction(trip3GardenTurretReady);

        PushBall trip3GardenShoot = new PushBall(stopper, intake);
        trip3GardenShoot.setName("trip3GardenShoot");
        trip3GardenShoot.setDependentActions(trip3GardenShooterReady, trip3GardenTurretReady);
        redThreeTipSoloAuto.addAction(trip3GardenShoot);

        ShooterStop trip3GardenShooterStop = new ShooterStop(trip3GardenShooterRun);
        trip3GardenShooterStop.setName("trip3GardenShooterStop");
        trip3GardenShooterStop.setDependentActions(trip3GardenShoot);
        redThreeTipSoloAuto.addAction(trip3GardenShooterStop);
        //END OF TRIP 3

        //TRIP 4
        IPurePursuitAction trip4LookAtBalls = new AdaptivePurePursuitAction(driveTrain);
        trip4LookAtBalls.setMaxTimeOutMS(2000); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        trip4LookAtBalls.setName("trip4LookAtBalls");
        trip4LookAtBalls.addPoint(450, 2321, 221.7);
        trip4LookAtBalls.setDependentActions(trip3GardenShoot);
        redThreeTipSoloAuto.addAction(trip4LookAtBalls);

        // Look from here, then drive to the best balls with the intake on, look again, until full (or nothing left / timeout).
        VisionPickupAction trip4GrabBalls = new VisionPickupAction(driveTrain, stopper, intake, allianceColor)
                .setTimeoutMs(visionPickupTimeoutMS);
        trip4GrabBalls.setName("trip4GrabBalls");
        trip4GrabBalls.setDependentActions(trip4LookAtBalls);
        redThreeTipSoloAuto.addAction(trip4GrabBalls);

        IPurePursuitAction trip4GoToShoot = new AdaptivePurePursuitAction(driveTrain);
        trip4GoToShoot.setMaxTimeOutMS(3000); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        trip4GoToShoot.setName("trip4GoToShoot");
        trip4GoToShoot.addPoint(714, 2952, 102.6);
        trip4GoToShoot.setDependentActions(trip4GrabBalls);
        redThreeTipSoloAuto.addAction(trip4GoToShoot);

        ShooterRun trip4ShooterRun = new ShooterRun(opModeUtilities, shooter, ShooterConfig.TARGET_POINT_1, new Point(714, 2952));
        trip4ShooterRun.setName("trip4ShooterRun");
        trip4ShooterRun.setDependentActions(trip4GoToShoot);
        redThreeTipSoloAuto.addAction(trip4ShooterRun);

        ShooterReady trip4ShooterReady = new ShooterReady(trip4ShooterRun);
        trip4ShooterReady.setName("trip4ShooterReady");
        trip4ShooterReady.setDependentActions(trip4GoToShoot);
        redThreeTipSoloAuto.addAction(trip4ShooterReady);

        TurretReady trip4TurretReady = new TurretReady(turretAutoAlign);
        trip4TurretReady.setName("trip4TurretReady");
        trip4TurretReady.setDependentActions(trip4GoToShoot);
        redThreeTipSoloAuto.addAction(trip4TurretReady);

        PushBall trip4Shoot = new PushBall(stopper, intake);
        trip4Shoot.setName("trip4Shoot");
        trip4Shoot.setDependentActions(trip4ShooterReady, trip4TurretReady);
        redThreeTipSoloAuto.addAction(trip4Shoot);

        ShooterStop trip4ShooterStop = new ShooterStop(trip4ShooterRun);
        trip4ShooterStop.setName("trip4ShooterStop");
        trip4ShooterStop.setDependentActions(trip4Shoot);
        redThreeTipSoloAuto.addAction(trip4ShooterStop);

        // Before parking: stop everything except the drivetrain.
        Action trip4StopEverything = new Action() {
            @Override
            protected void update() {
                stopEverythingButDrive();
                isDone = true;
            }
        };
        trip4StopEverything.setName("trip4StopEverything");
        trip4StopEverything.setDependentActions(trip4Shoot);
        redThreeTipSoloAuto.addAction(trip4StopEverything);

        IPurePursuitAction trip4Park = new AdaptivePurePursuitAction(driveTrain);
        trip4Park.setMaxTimeOutMS(3000); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        trip4Park.setName("trip4Park");
        trip4Park.addPoint(PARK_POINT.getX(), PARK_POINT.getY(), 90); // TODO-check: park heading
        trip4Park.setDependentActions(trip4StopEverything);
        redThreeTipSoloAuto.addAction(trip4Park);
        //END OF TRIP 4

        // FORCE PARK: with FORCE_PARK_MS_LEFT left in the match, drop everything and park
        IPurePursuitAction forcePark = new AdaptivePurePursuitAction(driveTrain);
        forcePark.setMaxTimeOutMS(3000); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        forcePark.setName("forcePark");
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
                stopEverythingButDrive();   // the auto actions are not updated any more, so nothing turns them back on
                KLog.d("RedThreeTipSoloAuto", "force park at " + matchTimer.milliseconds() + " ms");
            }

            if (forceParking) {
                forcePark.updateCheckDone();   // the auto actions are not updated any more
            } else {
                redThreeTipSoloAuto.updateCheckDone();
            }
            KLog.d("Odometry", () -> "Position: " + SharedData.getOdometryWheelIMUPosition());
        }
        shooter.stop();
        ballVision.close();
        cleanupRobot();
    }

    /** Aim the turret at the target from the start pose during init, so trip 0 shoots without waiting on the turret. */
    @Override
    protected void initLoop() {
        turretAutoAlign.updateCheckDone();
    }

    /** Shooter, intake and turret off: parking only needs the drivetrain. */
    private void stopEverythingButDrive() {
        shooter.stop();
        intake.stop();
        turretAutoAlign.stop();
        KLog.d("RedThreeTipSoloAuto", "stopped shooter, intake and turret before parking");
    }
}
