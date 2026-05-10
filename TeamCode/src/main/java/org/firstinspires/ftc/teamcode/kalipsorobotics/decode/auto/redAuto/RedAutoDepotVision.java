package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions.DepotRoundTrip;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions.RoundTripAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions.VisionRoundTripAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.cameraVision.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Vector3d;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveBrake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Tilter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.ShooterRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.ArtifactDetectionProcessor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.BlobSelectionStrategy;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionManager;

@Autonomous(name = "RedAutoDepotVision")
public class RedAutoDepotVision extends KOpMode {

    public final static double SHOOT_FAR_X = 150;
    public final static double SHOOT_FAR_Y = -50;
    Point farLaunchPoint = new Point(SHOOT_FAR_X, SHOOT_FAR_Y);
    Point thirdLaunchPoint = new Point(SHOOT_FAR_X, SHOOT_FAR_Y + 100);
    Point firstShootPoint = new Point(0, 0);
    Point firstShotTargetPoint = new Point(Shooter.TARGET_POINT.getX(), Shooter.TARGET_POINT.getY() + 141.4213562373);
    // Mandatory lookout: robot always drives here first; vision fires on arrival and decides next move
    Point depotLookoutPoint = new Point(500, 1050);

    // Modules
    private DriveTrain driveTrain;
    private Shooter shooter;
    private Intake intake;
    private Stopper stopper;
    private Tilter tilter;
    private Turret turret;
    private TurretAutoAlign turretAutoAlign;
    private ShooterRun shooterRun;

    // Vision
    private ArtifactDetectionProcessor artifactProcessor;
    private VisionManager visionManager;
    private CameraIntrinsics cameraIntrinsics;

    // Action set
    private KActionSet autoDepot;

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.RED;
        SharedData.setAllianceColor(allianceColor);
        TurretConfig.TICKS_INIT_OFFSET = 0;
    }

    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        KLog.d("RedAutoDepotVision-Init", "Starting initializeRobot()");

        // Create modules
        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);

        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000);

        // Create odometry - starting at (0,0,0) like RedAutoDepot
        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);
        OpModeUtilities.runOdometryExecutorService(odoExecutorService, odometry);

        autoDepot = new KActionSet();
        KLog.d("RedAutoDepotVision-Init", "Creating intake, shooter, stopper modules");
        intake = new Intake(opModeUtilities);
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);
        tilter = new Tilter(opModeUtilities);
        shooterRun = new ShooterRun(opModeUtilities, shooter, 0, ShooterInterpolationConfig.MAX_HOOD);
        shooterRun.setShooterRunMode(ShooterRunMode.STOP);

        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);
        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, allianceColor);

        // Initialize vision
        artifactProcessor = new ArtifactDetectionProcessor();
        visionManager = new VisionManager.Builder(hardwareMap)
                .addProcessor(artifactProcessor)
                .build();

        cameraIntrinsics = CameraIntrinsics.Arducam.withMount(
                Math.toRadians(0),      // Mount angle (tilt)
                new Vector3d(0, 150, 100)  // Camera offset from robot center (x, y, z in mm)
        );

        KLog.d("RedAutoDepotVision-Init", "Finished initializeRobot()");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        // ----------------- FIRST SHOOT (same as RedAutoDepot) ----------------------

        RoundTripAction trip0 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake,
                firstShotTargetPoint.multiplyY(allianceColor.getPolarity()), firstShootPoint, 0, false, true);
        trip0.setName("trip0");
        trip0.getShooterReady().setName("ShooterReady_trip0");
        trip0.getMoveToBall().addPoint(0, 0, 0);
        trip0.setShouldShooterStop(false);
        autoDepot.addAction(trip0);

        // ----------------- TRIP 1 (spike mark - same as RedAutoDepot) ----------------------

        DepotRoundTrip trip1 = new DepotRoundTrip(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake,
                Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), farLaunchPoint.multiplyY(allianceColor.getPolarity()), 0, allianceColor);
        trip1.setName("trip1");
        trip1.getMoveToShoot().getShooterReady().setName("ShooterReady_trip1");
        trip1.setDependentActions(trip0);
        addPointsToTrip1SpikeMark(trip1);
        trip1.getMoveToShoot().setShouldShooterStop(false);
        autoDepot.addAction(trip1);

        // ----------------- TRIP 2 (corner with vision fallback) ----------------------

        VisionRoundTripAction trip2 = new VisionRoundTripAction.Builder(
                opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake)
            .setTargetPoint(Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()))
            .setLaunchPoint(thirdLaunchPoint.multiplyY(allianceColor.getPolarity()))
            .enableVision(artifactProcessor, cameraIntrinsics, BlobSelectionStrategy.CLOSEST_TO_ROBOT_WORLD)
            .setVisionLookoutPoint(depotLookoutPoint.multiplyY(allianceColor.getPolarity()))
            .build();
        trip2.setName("trip2_CornerVision");
        trip2.setDependentActions(trip1);

        // Drive to lookout first — vision decides at that point.
        // Fallback path used if no ball is seen from the lookout.
        trip2.getMoveToBall().clearPoints();
        trip2.getMoveToBall().addPoint(depotLookoutPoint.getX(), depotLookoutPoint.getY() * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip2.getMoveToBall().addPoint(-25, 1075 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip2.getMoveToBall().addPoint(250, 1075 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());

        trip2.getMoveToBall().setFinalSearchRadiusMM(150);
        trip2.setShouldShooterStop(false);
        trip2.getPurePursuitReadyShooting().setDistanceThresholdMM(150);
        trip2.getMoveToBall().setPathAngleToleranceDeg(45);
        trip2.getMoveToBall().setFinalAngleLockingThresholdDeg(45);
        autoDepot.addAction(trip2);

        // ----------------- TRIP 3+ (vision-guided retries) ----------------------

        // All retry trips: drive to lookout first, vision decides; fallback sweeps if nothing seen.

        VisionRoundTripAction trip3 = createVisionRetryTrip(trip2, "trip3_Vision");
        trip3.getMoveToBall().clearPoints();
        trip3.getMoveToBall().addPoint(depotLookoutPoint.getX(), depotLookoutPoint.getY() * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip3.getMoveToBall().addPoint(325, 1050 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip3.getMoveToBall().addPoint(55, 800 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip3.getMoveToBall().addPoint(55, 1050 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        autoDepot.addAction(trip3);

        VisionRoundTripAction trip4 = createVisionRetryTrip(trip3, "trip4_Vision");
        trip4.getMoveToBall().clearPoints();
        trip4.getMoveToBall().addPoint(depotLookoutPoint.getX(), depotLookoutPoint.getY() * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip4.getMoveToBall().addPoint(25, 1050 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip4.getMoveToBall().addPoint(325, 800 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip4.getMoveToBall().addPoint(325, 1050 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        autoDepot.addAction(trip4);

        // trip5 sweeps opposite direction to trip3 to cover different ground
        VisionRoundTripAction trip5 = createVisionRetryTrip(trip4, "trip5_Vision");
        trip5.getMoveToBall().clearPoints();
        trip5.getMoveToBall().addPoint(depotLookoutPoint.getX(), depotLookoutPoint.getY() * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip5.getMoveToBall().addPoint(55, 800 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip5.getMoveToBall().addPoint(325, 800 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip5.getMoveToBall().addPoint(325, 1050 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        autoDepot.addAction(trip5);

        VisionRoundTripAction trip6 = createVisionRetryTrip(trip5, "trip6_Vision");
        trip6.getMoveToBall().clearPoints();
        trip6.getMoveToBall().addPoint(depotLookoutPoint.getX(), depotLookoutPoint.getY() * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip6.getMoveToBall().addPoint(800, 1050 * allianceColor.getPolarity(), 65 * allianceColor.getPolarity());
        autoDepot.addAction(trip6);

        // ----------------- PARK (same as RedAutoDepot) ----------------------

        shooterRun.setDependentActions(trip6);
        autoDepot.addAction(shooterRun);

        IntakeStop stopIntake = new IntakeStop(intake);
        stopIntake.setName("stopIntake");
        stopIntake.setDependentActions(trip6);
        autoDepot.addAction(stopIntake);

        PurePursuitAction park = new PurePursuitAction(driveTrain);
        park.setName("park");
        park.setDependentActions(trip6);
        park.addPoint(170, 540 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        park.setMaxCheckDoneCounter(20);
        autoDepot.addAction(park);

        KLog.d("auto", "-------------DEPOT AUTO VISION STARTED-------------");
        tilter.getTilterLeft().setPosition(ModuleConfig.TILT_LEFT_UP_POS);
        tilter.getTilterRight().setPosition(ModuleConfig.TILT_RIGHT_UP_POS);
        stopper.setPosition(ModuleConfig.STOPPER_SERVO_CLOSED_POS);

        waitForStart();
        KLog.d("RedAutoDepotVision-Run", "After waitForStart() - starting autonomous loop");

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            autoDepot.updateCheckDone();
            turretAutoAlign.updateCheckDone();
            KLog.d("Odometry", () -> "Position: " + SharedData.getOdometryWheelIMUPosition());
        }

        KLog.d("Auto→TeleOp", "=== AUTO ENDING ===");
        KLog.d("Auto→TeleOp", () -> "Final position: " + SharedData.getOdometryWheelIMUPosition());
        KLog.d("RedAutoDepotVision-Run", "Calling cleanupRobot()");
        visionManager.close();
        cleanupRobot();
        KLog.d("Auto→TeleOp", () -> "After cleanup position: " + SharedData.getOdometryWheelIMUPosition());
    }

    // GO TO SPIKE MARK FIRST - copied from RedAutoDepot
    private void addPointsToTrip1SpikeMark(DepotRoundTrip trip1) {
        trip1.getMoveToDepot().clearPoints();
        trip1.getMoveToDepot().addPoint(650, 110 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getMoveToDepot().addPoint(650, 1050 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getMoveToDepot().addPoint(500, 1050 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
    }

    private VisionRoundTripAction createVisionRetryTrip(VisionRoundTripAction lastTrip, String name) {
        VisionRoundTripAction retryTrip = new VisionRoundTripAction.Builder(
                opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake)
            .setTargetPoint(Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()))
            .setLaunchPoint(farLaunchPoint.multiplyY(allianceColor.getPolarity()))
            .enableVision(artifactProcessor, cameraIntrinsics, BlobSelectionStrategy.CLOSEST_TO_ROBOT_WORLD)
            .setVisionLookoutPoint(depotLookoutPoint.multiplyY(allianceColor.getPolarity()))
            .build();
        retryTrip.setName(name);
        retryTrip.setDependentActions(lastTrip);
        retryTrip.getMoveToBall().setFinalSearchRadiusMM(150);
        retryTrip.getPurePursuitReadyShooting().setDistanceThresholdMM(150);
        retryTrip.getMoveToBall().setPathAngleToleranceDeg(45);
        retryTrip.getMoveToBall().setFinalAngleLockingThresholdDeg(45);
        retryTrip.getMoveToBall().setMaxTimeOutMS(10000);
        retryTrip.setShouldShooterStop(false);
        return retryTrip;
    }
}
