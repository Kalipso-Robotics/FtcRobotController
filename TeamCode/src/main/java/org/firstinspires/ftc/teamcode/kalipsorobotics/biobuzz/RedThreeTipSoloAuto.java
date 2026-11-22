package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.SetAutoDelayAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.WaitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath.AutoToBallsPathingAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.beambreak.BeamBreakCountAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision.VisionAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.Flower;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.FullFlowerAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeFullAction;
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
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.IPurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.Raytracer;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.TFLitePollenNectarDetector;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@Autonomous
public class RedThreeTipSoloAuto extends KOpMode {
    KActionSet redThreeTipSoloAuto;
    public DriveTrain driveTrain;
    Shooter shooter = null;
    Intake intake = null;
    Stopper stopper = null;
    Turret turret = null;
    FlowerMech flowerMech = null;
    TurretAutoAlign turretAutoAlign = null;
    VisionPortal visionPortal = null;
    BeamBreakCountAction beamBreakCountAction = null;

    final double waitForShooterRunMS = 0.0;
    final double lookForBallsTimeoutSec = 1.0;     // TODO-measure: how long to look before giving up
    final int fullBallCount = 4;                   // the robot holds 4 balls

    // Must match the webcam name in the robot configuration (same as PollenNectarLive)
    static final String WEBCAM_NAME = "Arducam";

    final Position ROBOT_START_POS = new Position(1371.6, 152.0, 90);//double check 90 or nega 90
    final Point PARK_POINT = new Point(392, 1371);


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
        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, ShooterConfig.TARGET_POINT_1);
        startAprilTagDetection(turret);
        beamBreakCountAction = new BeamBreakCountAction(opModeUtilities); // counts the balls inside the robot
        beamBreakCountAction.setName("beamBreakCountAction");

        // Camera for trip 4: the raytracer puts every ball it sees into SharedData.getBallInformation() (field mm).
        SharedData.resetBallInformation();
        TFLitePollenNectarDetector detector = new TFLitePollenNectarDetector(hardwareMap.appContext)
                .withMinConfidence(0.35f);
        new Raytracer(detector, VisionConfig.ARDUCAM);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(1280, 720)) // the raytracer needs 1280x720
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(detector)
                .enableLiveView(false)
                .build();
    }

    /**
     * Supplier for the trip 4 look actions. Done (returns true) when the robot is already full, or when the camera sees a ball:
     * then the closest ball goes into SharedData.setTargetClusterXY for AutoToBallsPathingAction.
     * Target null = nothing to grab, so the grab action finishes right away without driving.
     */
    private Boolean lookForBalls() {
        if (intake.getBallCount() >= fullBallCount) {
            SharedData.setTargetClusterXY(null);
            return true;
        }
        Position robot = new Position(SharedData.getOdometryWheelIMUPosition());
        List<BallInformation> balls = SharedData.getBallInformation();
        BallInformation closest = null;
        double closestDistance = Double.MAX_VALUE;
        for (BallInformation ball : balls) {
            double distance = Math.hypot(ball.x - robot.getX(), ball.y - robot.getY());
            if (distance < closestDistance) {
                closestDistance = distance;
                closest = ball;
            }
        }
        if (closest == null) {
            SharedData.setTargetClusterXY(null); // if this look times out, the grab does nothing
            return null;
        }
        SharedData.setTargetClusterXY(new Point(closest.x, closest.y));
        return true;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        // TRIP 0
        SetAutoDelayAction setAutoDelayAction = new SetAutoDelayAction(opModeUtilities, gamepad1);
        setAutoDelayAction.setName("setAutoDelayAction");
        while (!setAutoDelayAction.getIsDone() && opModeInInit()) {
            setAutoDelayAction.updateCheckDone();
        }

        WaitAction delayBeforeStart = new WaitAction(setAutoDelayAction.getTimeMS());
        delayBeforeStart.setName("delayBeforeStart");
        redThreeTipSoloAuto.addAction(delayBeforeStart);

        WaitAction trip0WaitUntilShooterRun = new WaitAction(waitForShooterRunMS);
        trip0WaitUntilShooterRun.setName("trip0WaitUntilShooterRun");
        trip0WaitUntilShooterRun.setDependentActions(delayBeforeStart);
        redThreeTipSoloAuto.addAction(trip0WaitUntilShooterRun);

        ShooterRun trip0ShooterRun = new ShooterRun(opModeUtilities, shooter, SharedData.getTargetPoint(), ROBOT_START_POS.toPoint());
        trip0ShooterRun.setName("trip0ShooterRun");
        trip0ShooterRun.setDependentActions(trip0WaitUntilShooterRun);
        redThreeTipSoloAuto.addAction(trip0ShooterRun);

        ShooterReady trip0ShooterReady = new ShooterReady(trip0ShooterRun);
        trip0ShooterReady.setName("trip0ShooterReady");
        trip0ShooterReady.setDependentActions(trip0ShooterRun);
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
        FullFlowerAction trip1FlowerIntake = new FullFlowerAction(stopper, intake, IntakeConfig.intakeBallTimeMS, IntakeConfig.intakePower, flowerMech, driveTrain, Flower.FLOWER_1);
        trip1FlowerIntake.setName("trip1FlowerIntake");
        trip1FlowerIntake.setDependentActions(trip0Shoot);
        redThreeTipSoloAuto.addAction(trip1FlowerIntake);

        IPurePursuitAction trip1GoToShoot = new PurePursuitAction(driveTrain);
        trip1GoToShoot.setName("trip1GoToShoot");
        trip1GoToShoot.addPoint(1037, 397, 289.1);
        trip1GoToShoot.setDependentActions(trip1FlowerIntake);
        redThreeTipSoloAuto.addAction(trip1GoToShoot);

        ShooterRun trip1ShooterRun = new ShooterRun(opModeUtilities, shooter, SharedData.getTargetPoint(), new Point(1037, 397));
        trip1ShooterRun.setName("trip1ShooterRun");
        trip1ShooterRun.setDependentActions(trip1GoToShoot);
        redThreeTipSoloAuto.addAction(trip1ShooterRun);

        ShooterReady trip1ShooterReady = new ShooterReady(trip1ShooterRun);
        trip1ShooterReady.setName("trip1ShooterReady");
        trip1ShooterReady.setDependentActions(trip1ShooterRun);
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
        FullFlowerAction trip2FlowerIntake = new FullFlowerAction(stopper, intake, IntakeConfig.intakeBallTimeMS, IntakeConfig.intakePower, flowerMech, driveTrain, Flower.FLOWER_2);
        trip2FlowerIntake.setName("trip2FlowerIntake");
        trip2FlowerIntake.setDependentActions(trip1Shoot);
        redThreeTipSoloAuto.addAction(trip2FlowerIntake);

        // TODO-check: added so the robot is at (1037, 397) for this shot, same as trip 1
        IPurePursuitAction trip2GoToShoot = new PurePursuitAction(driveTrain);
        trip2GoToShoot.setName("trip2GoToShoot");
        trip2GoToShoot.addPoint(1037, 397, 289.1);
        trip2GoToShoot.setDependentActions(trip2FlowerIntake);
        redThreeTipSoloAuto.addAction(trip2GoToShoot);

        ShooterRun trip2ShooterRun = new ShooterRun(opModeUtilities, shooter, SharedData.getTargetPoint(), new Point(1037, 397));
        trip2ShooterRun.setName("trip2ShooterRun");
        trip2ShooterRun.setDependentActions(trip2GoToShoot);
        redThreeTipSoloAuto.addAction(trip2ShooterRun);

        ShooterReady trip2ShooterReady = new ShooterReady(trip2ShooterRun);
        trip2ShooterReady.setName("trip2ShooterReady");
        trip2ShooterReady.setDependentActions(trip2ShooterRun);
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
        // shoot again from the same spot before going to the garden
        ShooterRun trip3ShooterRun = new ShooterRun(opModeUtilities, shooter, SharedData.getTargetPoint(), new Point(1037, 397));
        trip3ShooterRun.setName("trip3ShooterRun");
        trip3ShooterRun.setDependentActions(trip2ShooterStop);
        redThreeTipSoloAuto.addAction(trip3ShooterRun);

        ShooterReady trip3ShooterReady = new ShooterReady(trip3ShooterRun);
        trip3ShooterReady.setName("trip3ShooterReady");
        trip3ShooterReady.setDependentActions(trip3ShooterRun);
        redThreeTipSoloAuto.addAction(trip3ShooterReady);

        TurretReady trip3TurretReady = new TurretReady(turretAutoAlign);
        trip3TurretReady.setName("trip3TurretReady");
        trip3TurretReady.setDependentActions(trip2Shoot);
        redThreeTipSoloAuto.addAction(trip3TurretReady);

        PushBall trip3Shoot = new PushBall(stopper, intake);
        trip3Shoot.setName("trip3Shoot");
        trip3Shoot.setDependentActions(trip3ShooterReady, trip3TurretReady);
        redThreeTipSoloAuto.addAction(trip3Shoot);

        ShooterStop trip3ShooterStop = new ShooterStop(trip3ShooterRun);
        trip3ShooterStop.setName("trip3ShooterStop");
        trip3ShooterStop.setDependentActions(trip3Shoot);
        redThreeTipSoloAuto.addAction(trip3ShooterStop);

        IPurePursuitAction trip3ForceAngle = new PurePursuitAction(driveTrain);
        trip3ForceAngle.setName("trip3ForceAngle");
        trip3ForceAngle.addPoint(1027, 487, 108.5);
        trip3ForceAngle.setDependentActions(trip3Shoot);
        redThreeTipSoloAuto.addAction(trip3ForceAngle);

        IPurePursuitAction trip3GoToGarden = new PurePursuitAction(driveTrain);
        trip3GoToGarden.setName("trip3GoToGarden");
        trip3GoToGarden.addPoint(289, 2949, 102.6);
        trip3GoToGarden.setDependentActions(trip3ForceAngle);
        redThreeTipSoloAuto.addAction(trip3GoToGarden);

        IntakeFullAction trip3IntakeFullAction = new IntakeFullAction(stopper, intake, IntakeConfig.intakeBallTimeMS, IntakeConfig.intakePower);
        trip3IntakeFullAction.setName("trip3IntakeFullAction");
        trip3IntakeFullAction.setDependentActions(trip3GoToGarden);
        redThreeTipSoloAuto.addAction(trip3IntakeFullAction);

        IPurePursuitAction trip3IntakeGarden = new PurePursuitAction(driveTrain);
        trip3IntakeGarden.setName("trip3IntakeGarden");
        trip3IntakeGarden.addPoint(188, 3309, 90.0);
        trip3IntakeGarden.setDependentActions(trip3GoToGarden);
        redThreeTipSoloAuto.addAction(trip3IntakeGarden);

        // in and out a second time to make sure we got the balls
        IPurePursuitAction trip3GoToGarden2 = new PurePursuitAction(driveTrain);
        trip3GoToGarden2.setName("trip3GoToGarden2");
        trip3GoToGarden2.addPoint(289, 2949, 102.6);
        trip3GoToGarden2.setDependentActions(trip3IntakeGarden);
        redThreeTipSoloAuto.addAction(trip3GoToGarden2);

        IPurePursuitAction trip3IntakeGarden2 = new PurePursuitAction(driveTrain);
        trip3IntakeGarden2.setName("trip3IntakeGarden2");
        trip3IntakeGarden2.addPoint(188, 3309, 90.0);
        trip3IntakeGarden2.setDependentActions(trip3GoToGarden2);
        redThreeTipSoloAuto.addAction(trip3IntakeGarden2);

        IPurePursuitAction trip3GoToShoot = new PurePursuitAction(driveTrain);
        trip3GoToShoot.setName("trip3GoToShoot");
        trip3GoToShoot.addPoint(300, 2949, 102.6);
        trip3GoToShoot.setDependentActions(trip3IntakeGarden2);
        redThreeTipSoloAuto.addAction(trip3GoToShoot);

        ShooterRun trip3GardenShooterRun = new ShooterRun(opModeUtilities, shooter, SharedData.getTargetPoint(), new Point(300, 2949));
        trip3GardenShooterRun.setName("trip3GardenShooterRun");
        trip3GardenShooterRun.setDependentActions(trip3GoToShoot);
        redThreeTipSoloAuto.addAction(trip3GardenShooterRun);

        ShooterReady trip3GardenShooterReady = new ShooterReady(trip3GardenShooterRun);
        trip3GardenShooterReady.setName("trip3GardenShooterReady");
        trip3GardenShooterReady.setDependentActions(trip3GardenShooterRun);
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
        IPurePursuitAction trip4LookAtBalls = new PurePursuitAction(driveTrain);
        trip4LookAtBalls.setName("trip4LookAtBalls");
        trip4LookAtBalls.addPoint(450, 2321, 221.7);
        trip4LookAtBalls.setDependentActions(trip3GardenShoot);
        redThreeTipSoloAuto.addAction(trip4LookAtBalls);

        VisionAction<Boolean> trip4LookForBalls = new VisionAction<>(this::lookForBalls, lookForBallsTimeoutSec);
        trip4LookForBalls.setName("trip4LookForBalls");
        trip4LookForBalls.setDependentActions(trip4LookAtBalls);
        redThreeTipSoloAuto.addAction(trip4LookForBalls);

        AutoToBallsPathingAction trip4GrabBalls = new AutoToBallsPathingAction(driveTrain, stopper, intake, true);
        trip4GrabBalls.setName("trip4GrabBalls");
        trip4GrabBalls.setDependentActions(trip4LookForBalls);
        redThreeTipSoloAuto.addAction(trip4GrabBalls);

        VisionAction<Boolean> trip4LookForBalls2 = new VisionAction<>(this::lookForBalls, lookForBallsTimeoutSec);
        trip4LookForBalls2.setName("trip4LookForBalls2");
        trip4LookForBalls2.setDependentActions(trip4GrabBalls);
        redThreeTipSoloAuto.addAction(trip4LookForBalls2);

        AutoToBallsPathingAction trip4GrabBalls2 = new AutoToBallsPathingAction(driveTrain, stopper, intake, true);
        trip4GrabBalls2.setName("trip4GrabBalls2");
        trip4GrabBalls2.setDependentActions(trip4LookForBalls2);
        redThreeTipSoloAuto.addAction(trip4GrabBalls2);

        IPurePursuitAction trip4GoToShoot = new PurePursuitAction(driveTrain);
        trip4GoToShoot.setName("trip4GoToShoot");
        trip4GoToShoot.addPoint(300, 2949, 102.6);
        trip4GoToShoot.setDependentActions(trip4GrabBalls2);
        redThreeTipSoloAuto.addAction(trip4GoToShoot);

        ShooterRun trip4ShooterRun = new ShooterRun(opModeUtilities, shooter, SharedData.getTargetPoint(), new Point(300, 2949));
        trip4ShooterRun.setName("trip4ShooterRun");
        trip4ShooterRun.setDependentActions(trip4GoToShoot);
        redThreeTipSoloAuto.addAction(trip4ShooterRun);

        ShooterReady trip4ShooterReady = new ShooterReady(trip4ShooterRun);
        trip4ShooterReady.setName("trip4ShooterReady");
        trip4ShooterReady.setDependentActions(trip4ShooterRun);
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

        IPurePursuitAction trip4Park = new PurePursuitAction(driveTrain);
        trip4Park.setName("trip4Park");
        trip4Park.addPoint(PARK_POINT.getX(), PARK_POINT.getY(), 90); // TODO-check: park heading
        trip4Park.setDependentActions(trip4Shoot);
        redThreeTipSoloAuto.addAction(trip4Park);
        //END OF TRIP 4

        waitForStartPrecomputingPaths();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            intake.update();
            beamBreakCountAction.updateCheckDone(); // publishes the ball count that lookForBalls() reads
            turretAutoAlign.updateCheckDone(); // the turret only moves while its action is updated
            redThreeTipSoloAuto.updateCheckDone();
            KLog.d("Odometry", () -> "Position: " + SharedData.getOdometryWheelIMUPosition());
        }
        shooter.stop();
        visionPortal.close();
        cleanupRobot();
    }
}
