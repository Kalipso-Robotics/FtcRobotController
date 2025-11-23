package com.kalipsorobotics.decode;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.SetAutoDelayAction;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.actions.autoActions.pathActions.RoundTripAction;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.intake.IntakeFullAction;
import com.kalipsorobotics.actions.intake.IntakeReverse;
import com.kalipsorobotics.actions.intake.IntakeRun;
import com.kalipsorobotics.actions.intake.IntakeStop;
import com.kalipsorobotics.actions.revolverActions.DetectColorsAction;
import com.kalipsorobotics.actions.revolverActions.RevolverTeleOp;
import com.kalipsorobotics.actions.shooter.ShootAllAction;
import com.kalipsorobotics.actions.shooter.ShooterRun;
import com.kalipsorobotics.actions.shooter.ShooterStop;
import com.kalipsorobotics.actions.shooter.pusher.PushBall;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.actions.turret.TurretConfig;
import com.kalipsorobotics.cameraVision.AllianceSetup;
import com.kalipsorobotics.cameraVision.MotifCamera;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.MotifColor;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.TripleColorSensor;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.modules.shooter.LaunchPosition;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KTeleOp;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedAutoNearZone")
public class RedAutoNear extends KTeleOp {
    KActionSet redAutoNear;

    protected AllianceSetup allianceSetup = AllianceSetup.RED;
    final double FIRST_SHOOT_X = 2598;
    final double FIRST_SHOOT_Y = 441.38;
    final double SHOOT_NEAR_X = 2130; //2400
    final double SHOOT_NEAR_Y = 135; //300
    final double LEVER_X = 1700; // 1700,
    final double LEVER_Y = 1080; //1125, 1040
    final double DEPOT_X = 150; // final ending 0 x
    final double DEPOT_Y = 1100  ; // final ending 1,165.2 , 135degree heading
    final double FIRST_BALL_X = 775;
    final double FIRST_BALL_Y = 1085;
    final double SECOND_BALL_X = 1300;
    final double SECOND_BALL_Y = 1120;
    final double THIRD_BALL_X = 1900;
    final double THIRD_BALL_Y = 670;
    private DriveTrain driveTrain;
    TripleColorSensor colorSensors = null;
    Shooter shooter = null;
    Intake intake = null;
    Stopper stopper = null;
    ShooterRun shooterRun = null;
    ShooterStop shooterStop = null;
    ShootAllAction shootAction = null;
    LaunchPosition launchPosition = LaunchPosition.AUTO;
    PushBall pushBall = null;
    Turret turret = null;
    IntakeRun intakeRun = null;
    IntakeStop intakeStop = null;
    IntakeFullAction intakeFullAction = null;
    RevolverTeleOp revolverTeleOp = null;
    IntakeReverse intakeReverse = null;
    DriveAction driveAction = null;
    TurretAutoAlign turretAutoAlign = null;
    DetectColorsAction detectColorsAction = null;
    MotifCamera.MotifPattern testingMotif;

    @Override
    protected void initializeRobot() {
        super.initializeRobot();
        allianceSetup = AllianceSetup.RED;

        // Create your modules
        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000); // Optional: let hardware initialize

        // Create odometry
        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, 3028.98, 746.18, -2.4137); //3015.93, 765.86, -2.4030
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);

        colorSensors = new TripleColorSensor(opModeUtilities);

        redAutoNear = new KActionSet();
        intake = new Intake(opModeUtilities);
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);

        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);
        intakeRun = new IntakeRun(intake);
        intakeStop = new IntakeStop(intake);
        intakeReverse = new IntakeReverse(intake);
        intakeFullAction = new IntakeFullAction(stopper, intake, 10);

        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, TurretConfig.RED_X_INIT_SETUP, TurretConfig.RED_Y_INIT_SETUP * allianceSetup.getPolarity());

        detectColorsAction = new DetectColorsAction(colorSensors, opModeUtilities);

        //todo just fed in testing motif pattern change later
        testingMotif = new MotifCamera.MotifPattern(MotifColor.PURPLE, MotifColor.PURPLE, MotifColor.GREEN);
//        testingMotif = new ObiliskDetection.MotifPattern(MotifColor.PURPLE, MotifColor.PURPLE, MotifColor.GREEN);
//        fullShootMotifAction = new FullShootMotifAction(revolver, shooter, testingMotif, colorSensors, opModeUtilities);

        shooterRun = new ShooterRun(shooter, Shooter.RED_TARGET_FROM_FAR, LaunchPosition.AUTO);
        shootAction = new ShootAllAction(stopper, intake, shooter, Shooter.RED_TARGET_FROM_FAR);
        shooterStop = new ShooterStop(shooterRun);
        pushBall = new PushBall(stopper, intake, shooter);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        Point nearLaunchPoint =  new Point(SHOOT_NEAR_X, SHOOT_NEAR_Y * allianceSetup.getPolarity());
        Point firstShootPoint = new Point(FIRST_SHOOT_X, FIRST_SHOOT_Y * allianceSetup.getPolarity());

        SetAutoDelayAction setAutoDelayAction = new SetAutoDelayAction(opModeUtilities, gamepad1);
        setAutoDelayAction.setName("setAutoDelayAction");

        while(!setAutoDelayAction.getIsDone() && opModeInInit()) {
            setAutoDelayAction.updateCheckDone();
        }

        WaitAction delayBeforeStart = new WaitAction(setAutoDelayAction.getTimeMS());
        delayBeforeStart.setName("delayBeforeStart");
        redAutoNear.addAction(delayBeforeStart);


        // ----------------- FIRST SHOOT ----------------------
        RoundTripAction trip0 = new RoundTripAction(opModeUtilities, driveTrain, shooter, stopper, intake, Shooter.RED_TARGET_FROM_FAR.multiplyY(allianceSetup.getPolarity()), firstShootPoint, 0, false);
        trip0.setName("trip0");
        trip0.getMoveToBall().addPoint(FIRST_SHOOT_X, FIRST_SHOOT_Y*allianceSetup.getPolarity(), -138.29);
        redAutoNear.addAction(trip0);

        // ----------------- TRIP 1 ----------------------

        RoundTripAction trip1 = new RoundTripAction(opModeUtilities, driveTrain, shooter, stopper, intake, Shooter.RED_TARGET_FROM_FAR.multiplyY(allianceSetup.getPolarity()), nearLaunchPoint, 0);
        trip1.setName("trip1");
        trip1.getMoveToBall().addPoint(1950, 385 * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        trip1.getMoveToBall().addPoint(1950, 800 * allianceSetup.getPolarity() , 90 * allianceSetup.getPolarity());
        trip1.getMoveToBall().addPoint(1950, 1015 * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        // move to hit lever
        trip1.getMoveToBall().addPoint(1685, 1055 * allianceSetup.getPolarity(), -20);
        trip1.getMoveToBall().addPoint(nearLaunchPoint.getX(), nearLaunchPoint.getY() * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        trip1.setDependentActions(trip0);
        redAutoNear.addAction(trip1);

        // ----------------- TRIP 2 ----------------------

        RoundTripAction trip2 = new RoundTripAction(opModeUtilities, driveTrain, shooter, stopper, intake, Shooter.RED_TARGET_FROM_FAR.multiplyY(allianceSetup.getPolarity()), nearLaunchPoint, 0);
        trip2.setName("trip2");
        trip2.getMoveToBall().addPoint(1280, 200 * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        trip2.getMoveToBall().addPoint(1280, 820 * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        trip2.getMoveToBall().addPoint(1280, 1220 * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        trip2.getMoveToBall().addPoint(1280, 820 * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        trip2.getMoveToBall().addPoint(nearLaunchPoint.getX(), nearLaunchPoint.getY() * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        trip2.setDependentActions(trip1);
        redAutoNear.addAction(trip2);

        // ----------------- TRIP 3 ----------------------

        RoundTripAction trip3 = new RoundTripAction(opModeUtilities, driveTrain, shooter, stopper, intake, Shooter.RED_TARGET_FROM_FAR.multiplyY(allianceSetup.getPolarity()), nearLaunchPoint, 500);
        trip3.setName("trip3");
        trip3.getMoveToBall().addPoint(680, 315 * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        // move to intake
        trip3.getMoveToBall().addPoint(680, 835 * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        trip3.getMoveToBall().addPoint(680, 1220 * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        trip3.getMoveToBall().addPoint(1040, 800 * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        trip3.getMoveToBall().addPoint(nearLaunchPoint.getX(), nearLaunchPoint.getY() * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        trip3.setDependentActions(trip2);
        redAutoNear.addAction(trip3);

        // ----------------- PARK ----------------------

        PurePursuitAction park = new PurePursuitAction(driveTrain);
        park.setName("park");
        park.setDependentActions(trip3);
        park.addPoint(SHOOT_NEAR_X - 700, (SHOOT_NEAR_Y) * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        park.setMaxCheckDoneCounter(20);
        redAutoNear.addAction(park);

        turretAutoAlign.initBlocking();


        waitForStart();
        long startTime = System.currentTimeMillis();
        int loopCount = 0;
        while (opModeIsActive()) {
            loopCount++;
            double elapsedSec = (System.currentTimeMillis() - startTime) / 1000.0;

            // Log overall progress every 500ms
            if (loopCount % 25 == 0) {  // Assuming ~50Hz loop rate
                KLog.d("AutoProgress", String.format("=== RedAutoNear - Time: %.1fs, Loop: %d, AutoDone: %b ===",
                    elapsedSec, loopCount, redAutoNear.getIsDone()));
                KLog.d("AutoProgress", String.format("Trips -> Trip1: %s, Trip2: %s, Trip3: %s, Park: %s",
                    trip1.getIsDone() ? "✓" : "...",
                    trip2.getIsDone() ? "✓" : "...",
                    trip3.getIsDone() ? "✓" : "...",
                    park.getIsDone() ? "✓" : "..."));
                KLog.d("AutoProgress", "NOTE: Trip3 is commented out in code - Park depends on Trip3!");
            }

            redAutoNear.updateCheckDone();
            turretAutoAlign.updateCheckDone();
            KLog.d("Odometry", "Position: " + SharedData.getOdometryPosition());
        }
        cleanupRobot();
    }
}