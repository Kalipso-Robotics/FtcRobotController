package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.SetAutoDelayAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.WaitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions.RoundTripAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions.VisionRoundTripAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.cameraVision.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Vector3d;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.ArtifactDetectionProcessor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionManager;

@Autonomous(name = "RedAutoNearVision")
public class RedAutoDepotVision extends KOpMode {

    // Launch positions
    private final Point FIRST_SHOOT_POINT = new Point(2400, 128);
    private final Point NEAR_LAUNCH_POINT = new Point(2050, 100);
    private final Point FIRST_SHOT_TARGET = new Point(
            Shooter.TARGET_POINT.getX() - 141.4213562373,
            Shooter.TARGET_POINT.getY() - 141.4213562373
    );

    // Modules
    private DriveTrain driveTrain;
    private Shooter shooter;
    private Intake intake;
    private Stopper stopper;
    private Turret turret;
    private TurretAutoAlign turretAutoAlign;

    // Vision
    private ArtifactDetectionProcessor artifactProcessor;
    private VisionManager visionManager;
    private CameraIntrinsics cameraIntrinsics;

    // Action set
    private KActionSet autoActionSet;

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.RED;
        SharedData.setAllianceColor(allianceColor);
        TurretConfig.TICKS_INIT_OFFSET = (int) -Math.round(
                (TurretConfig.TICKS_PER_ROTATION * TurretConfig.BIG_TO_SMALL_PULLEY) / 2
        );
    }

    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        // Initialize modules
        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000);

        // Initialize odometry
        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule,
                3060, 712 * allianceColor.getPolarity(), -2.4049 * allianceColor.getPolarity());
        OpModeUtilities.runOdometryExecutorService(odoExecutorService, odometry);

        // Initialize robot modules
        autoActionSet = new KActionSet();
        intake = new Intake(opModeUtilities);
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);

        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);
        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, allianceColor);
        turretAutoAlign.setToleranceDeg(3);

        // Initialize vision
        artifactProcessor = new ArtifactDetectionProcessor();
        visionManager = new VisionManager.Builder(hardwareMap)
                .addProcessor(artifactProcessor)
                .build();

        // Camera intrinsics - UPDATE THESE WITH YOUR CALIBRATION
        cameraIntrinsics = CameraIntrinsics.Arducam.withMount(
                Math.toRadians(0),      // Mount angle (tilt)
                new Vector3d(0, 150, 100)  // Camera offset from robot center (x, y, z in mm)
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        // Auto delay configuration
        SetAutoDelayAction setAutoDelayAction = new SetAutoDelayAction(opModeUtilities, gamepad1);
        setAutoDelayAction.setName("setAutoDelayAction");

        WaitAction delayBeforeStart = new WaitAction(setAutoDelayAction.getTimeMS());
        delayBeforeStart.setName("delayBeforeStart");
        autoActionSet.addAction(delayBeforeStart);

        // ======================== PHASE 1: SPIKE MARK (Manual Waypoints) ========================

        // Trip 0: First shot from starting position
        RoundTripAction trip0 = new RoundTripAction(
                opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake,
                FIRST_SHOT_TARGET.multiplyY(allianceColor.getPolarity()),
                FIRST_SHOOT_POINT.multiplyY(allianceColor.getPolarity()),
                0, true
        );
        trip0.setName("trip0_FirstShot");
        trip0.getMoveToBall().clearPoints();
        trip0.getMoveToBall().addPoint(
                FIRST_SHOOT_POINT.getX(),
                FIRST_SHOOT_POINT.getY() * allianceColor.getPolarity(),
                -138.29 * allianceColor.getPolarity()
        );
        trip0.setDependentActions(delayBeforeStart);
        trip0.setShouldShooterStop(false);
        trip0.getPurePursuitReadyShooting().setDistanceThresholdMM(250);
        trip0.getMoveToBall().setFinalAngleLockingThresholdDeg(50);
        autoActionSet.addAction(trip0);

        // Trip 1: Spike mark with manual waypoints (drive to known position)
        RoundTripAction trip1 = new RoundTripAction(
                opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake,
                Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()),
                NEAR_LAUNCH_POINT.multiplyY(allianceColor.getPolarity()),
                0
        );
        trip1.setName("trip1_SpikeManual");
        trip1.getMoveToBall().clearPoints();
        // Drive to spike mark area
        trip1.getMoveToBall().addPoint(1950, 175 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getMoveToBall().addPoint(1950, 650 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        // Hit lever
        trip1.getMoveToBall().addPoint(1725, 925 * allianceColor.getPolarity(), -10);
        trip1.getMoveToBall().addPoint(1725, 1060 * allianceColor.getPolarity(), -10);
        trip1.getMoveToBall().addPoint(1725, 850 * allianceColor.getPolarity(), 45 * allianceColor.getPolarity());
        // Move to shoot
        trip1.getMoveToBall().addPoint(
                NEAR_LAUNCH_POINT.getX(),
                NEAR_LAUNCH_POINT.getY() * allianceColor.getPolarity(),
                45 * allianceColor.getPolarity()
        );
        trip1.getMoveToBall().setFinalAngleLockingThresholdDeg(45);
        trip1.setShouldShooterStop(false);
        trip1.getMoveToBall().setFinalSearchRadiusMM(200);
        trip1.getPurePursuitReadyShooting().setDistanceThresholdMM(300);
        trip1.setDependentActions(trip0);
        autoActionSet.addAction(trip1);

        // ======================== PHASE 2: VISION-GUIDED COLLECTION ========================

        // Trip 2: Vision-guided ball collection (no color filter, any ball)
        VisionRoundTripAction trip2Vision = new VisionRoundTripAction.Builder(
                opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake)
            .setTargetPoint(Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()))
            .setLaunchPoint(NEAR_LAUNCH_POINT.multiplyY(allianceColor.getPolarity()))
            .enableVision(artifactProcessor, cameraIntrinsics)  // Uses CLOSEST_TO_CAMERA_CENTER
            .build();
        trip2Vision.setName("trip2_Vision");
        trip2Vision.setShouldShooterStop(false);
        trip2Vision.getPurePursuitReadyShooting().setDistanceThresholdMM(300);
        trip2Vision.getMoveToBall().setFinalAngleLockingThresholdDeg(45);
        trip2Vision.setDependentActions(trip1);
        autoActionSet.addAction(trip2Vision);

        // Trip 3: Another vision-guided collection
        VisionRoundTripAction trip3Vision = new VisionRoundTripAction.Builder(
                opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake)
            .setTargetPoint(Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()))
            .setLaunchPoint(NEAR_LAUNCH_POINT.multiplyY(allianceColor.getPolarity()))
            .enableVision(artifactProcessor, cameraIntrinsics)
            .build();
        trip3Vision.setName("trip3_Vision");
        trip3Vision.setShouldShooterStop(false);
        trip3Vision.getPurePursuitReadyShooting().setDistanceThresholdMM(300);
        trip3Vision.setDependentActions(trip2Vision);
        autoActionSet.addAction(trip3Vision);

        // Trip 4: Final vision collection
        VisionRoundTripAction trip4Vision = new VisionRoundTripAction.Builder(
                opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake)
            .setTargetPoint(Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()))
            .setLaunchPoint(NEAR_LAUNCH_POINT.multiplyY(allianceColor.getPolarity()))
            .enableVision(artifactProcessor, cameraIntrinsics)
            .build();
        trip4Vision.setName("trip4_Vision");
        trip4Vision.setShouldShooterStop(true);  // Stop shooter after last trip
        trip4Vision.setDependentActions(trip3Vision);
        autoActionSet.addAction(trip4Vision);

        // ======================== INIT LOOP ========================

        while (!setAutoDelayAction.getIsDone() && opModeInInit()) {
            setAutoDelayAction.updateCheckDone();

            // Show vision status during init
            telemetry.addLine("=== Red Auto Near Vision ===");
            telemetry.addData("Purple Detected", artifactProcessor.hasPurpleBlob());
            telemetry.addData("Green Detected", artifactProcessor.hasGreenBlob());
            telemetry.addLine();
            telemetry.addLine("Phase 1: Manual waypoints to spike");
            telemetry.addLine("Phase 2: Vision-guided ball collection");
            telemetry.addData("Strategy", "CLOSEST_TO_CAMERA_CENTER");
            telemetry.update();
        }

        // ======================== MAIN LOOP ========================

        KLog.d("auto", "--------------RED AUTO NEAR VISION STARTED-------------");
        waitForStart();

        long startTime = System.currentTimeMillis();
        int loopCount = 0;

        while (opModeIsActive()) {
            updateSensorData();
            loopCount++;
            double elapsedSec = (System.currentTimeMillis() - startTime) / 1000.0;

            // Update actions
            autoActionSet.update();

            // Telemetry every 500ms
            if (loopCount % 25 == 0) {
                telemetry.addLine("=== Red Auto Near Vision ===");
                telemetry.addData("Time", String.format("%.1fs", elapsedSec));
                telemetry.addData("Auto Complete", autoActionSet.getIsDone());
                telemetry.addLine();

                // Show vision detection status
                if (trip2Vision.isUsingVision() && trip2Vision.hasDetectedBall()) {
                    Point ballPos = trip2Vision.getDetectedBallWorldPos();
                    telemetry.addData("Trip 2 Ball", String.format("(%.0f, %.0f)", ballPos.getX(), ballPos.getY()));
                }
                if (trip3Vision.isUsingVision() && trip3Vision.hasDetectedBall()) {
                    Point ballPos = trip3Vision.getDetectedBallWorldPos();
                    telemetry.addData("Trip 3 Ball", String.format("(%.0f, %.0f)", ballPos.getX(), ballPos.getY()));
                }
                if (trip4Vision.isUsingVision() && trip4Vision.hasDetectedBall()) {
                    Point ballPos = trip4Vision.getDetectedBallWorldPos();
                    telemetry.addData("Trip 4 Ball", String.format("(%.0f, %.0f)", ballPos.getX(), ballPos.getY()));
                }

                telemetry.update();
            }

            // Exit when done
            if (autoActionSet.getIsDone()) {
                KLog.d("auto", String.format("RedAutoNearVision completed in %.1fs", elapsedSec));
                break;
            }
        }

        // Cleanup
        visionManager.close();
    }
}
