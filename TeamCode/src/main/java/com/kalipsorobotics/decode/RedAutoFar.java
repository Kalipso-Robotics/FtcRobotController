package com.kalipsorobotics.decode;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.intake.IntakeFullAction;
import com.kalipsorobotics.actions.intake.IntakeReverse;
import com.kalipsorobotics.actions.intake.IntakeRun;
import com.kalipsorobotics.actions.intake.IntakeStop;
import com.kalipsorobotics.actions.revolverActions.DetectColorsAction;
import com.kalipsorobotics.actions.revolverActions.RevolverTeleOp;
import com.kalipsorobotics.actions.shooter.pusher.PushBall;
import com.kalipsorobotics.actions.shooter.ShootAllAction;
import com.kalipsorobotics.actions.shooter.ShooterReady;
import com.kalipsorobotics.actions.shooter.ShooterStop;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.cameraVision.AllianceSetup;
import com.kalipsorobotics.cameraVision.MotifCamera;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.MotifColor;
import com.kalipsorobotics.modules.Revolver;
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


@Autonomous(name = "RedAutoFarZone")
public class RedAutoFar extends KTeleOp {
    KActionSet redAutoNear;

    protected AllianceSetup allianceSetup = AllianceSetup.RED;

    protected final Point ROBOT_START_POINT_RED = Shooter.RED_TARGET_FROM_NEAR;
    final double DISTANCE_BETWEEN_BALLS = 630;
    final double DISTANCE_TO_BALL_FROM_ORIGIN = 350;
    final double SHOOT_FAR_X = 10;
    final double SHOOT_FAR_Y = -20  * allianceSetup.getPolarity();
    final double LEVER_X = 1574.8;
    final double LEVER_Y = 1092  * allianceSetup.getPolarity();
    final double DEPOT_X = 0;
    final double DEPOT_Y = 630  * allianceSetup.getPolarity();
    final double FIRST_BALL_X = 304.8;
    final double FIRST_BALL_Y = DISTANCE_TO_BALL_FROM_ORIGIN  * allianceSetup.getPolarity();
    final double SECOND_BALL_X = 304.8 + DISTANCE_BETWEEN_BALLS ;
    final double SECOND_BALL_Y = DISTANCE_TO_BALL_FROM_ORIGIN  * allianceSetup.getPolarity();
    final double THIRD_BALL_X = 304.8 + DISTANCE_BETWEEN_BALLS*2;
    final double THIRD_BALL_Y = DISTANCE_TO_BALL_FROM_ORIGIN  * allianceSetup.getPolarity();
    private DriveTrain driveTrain;
    TripleColorSensor colorSensors = null;
    Shooter shooter = null;
    Intake intake = null;
    Stopper stopper = null;
    ShooterReady shooterReady = null;
    ShooterStop shooterStop = null;
    ShootAllAction shootAction = null;

    LaunchPosition launchPosition = LaunchPosition.AUTO;
    PushBall pushBall = null;
    Revolver revolver = null;
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
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000); // Optional: let hardware initialize

        // Create odometry
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);

        OpModeUtilities.runOdometryExecutorService(executorService, odometry);

        colorSensors = new TripleColorSensor(opModeUtilities);

        redAutoNear = new KActionSet();
        intake = new Intake(opModeUtilities);
        shooter = new Shooter(opModeUtilities);
        revolver = new Revolver(opModeUtilities);
        stopper = new Stopper(opModeUtilities);
        revolver.getRevolverServo().setPosition(Revolver.REVOLVER_INDEX_0);

        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);
        intakeRun = new IntakeRun(intake);
        intakeStop = new IntakeStop(intake);
        intakeReverse = new IntakeReverse(intake);
        intakeFullAction = new IntakeFullAction(intake);

        revolverTeleOp = new RevolverTeleOp(revolver, false);

        turretAutoAlign = new TurretAutoAlign(turret, TurretAutoAlign.RED_X_INIT_SETUP, TurretAutoAlign.RED_Y_INIT_SETUP * allianceSetup.getPolarity());

        detectColorsAction = new DetectColorsAction(colorSensors, opModeUtilities);

        //todo just fed in testing motif pattern change later
        testingMotif = new MotifCamera.MotifPattern(MotifColor.PURPLE, MotifColor.PURPLE, MotifColor.GREEN);
//        testingMotif = new ObiliskDetection.MotifPattern(MotifColor.PURPLE, MotifColor.PURPLE, MotifColor.GREEN);
//        fullShootMotifAction = new FullShootMotifAction(revolver, shooter, testingMotif, colorSensors, opModeUtilities);

        shooterReady = new ShooterReady(shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO);
        shootAction = new ShootAllAction(stopper, intake, shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO);
        shooterStop = new ShooterStop(shooter);
        pushBall = new PushBall(stopper, intake);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        // auto code here
        ShootAllAction firstShoot = new ShootAllAction(stopper, intake, shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO);
        firstShoot.setName("firstShoot");
        redAutoNear.addAction(firstShoot);

        PurePursuitAction moveToDepotBalls = new PurePursuitAction(driveTrain);
        moveToDepotBalls.setName("moveToDepotBalls");
        moveToDepotBalls.setDependentActions(firstShoot);
        moveToDepotBalls.addPoint(DEPOT_X, DEPOT_Y, 90 * allianceSetup.getPolarity());
        // move to intake
        moveToDepotBalls.addPoint(DEPOT_X, DEPOT_Y + (520 * allianceSetup.getPolarity()), 90 * allianceSetup.getPolarity());
        redAutoNear.addAction(moveToDepotBalls);

        IntakeFullAction first3Intake = new IntakeFullAction(intake);
        first3Intake.setName("first3Intake");
        first3Intake.setDependentActions(firstShoot);
        redAutoNear.addAction(first3Intake);

        PurePursuitAction moveToFarShoot = new PurePursuitAction(driveTrain);
        moveToFarShoot.setName("moveToFarShoot");
        moveToFarShoot.setDependentActions(first3Intake, moveToDepotBalls);
        moveToFarShoot.addPoint(SHOOT_FAR_X, SHOOT_FAR_Y, 90 * allianceSetup.getPolarity());
        redAutoNear.addAction(moveToFarShoot);
//
//        ShootAllAction secondShoot = new ShootAllAction(stopper, intake, shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO);
//        secondShoot.setName("secondShoot");
//        secondShoot.setDependentActions(moveToFarShoot);
//        redAutoNear.addAction(secondShoot);
//
//        PurePursuitAction moveToLeverBalls = new PurePursuitAction(driveTrain);
//        moveToLeverBalls.setName("moveToLeverBalls");
//        moveToLeverBalls.setDependentActions(secondShoot);
//        moveToLeverBalls.addPoint(SECOND_BALL_X, SECOND_BALL_Y, 90 * allianceSetup.getPolarity());
//        // move to intake
//        moveToLeverBalls.addPoint(SECOND_BALL_X, SECOND_BALL_Y + (520 * allianceSetup.getPolarity()), 90 * allianceSetup.getPolarity());
//        redAutoNear.addAction(moveToLeverBalls);
//
//        IntakeFullAction second6Intake = new IntakeFullAction(intake);
//        second6Intake.setName("second6Intake");
//        second6Intake.setDependentActions(secondShoot);
//        redAutoNear.addAction(second6Intake);
//
//        PurePursuitAction hitLever = new PurePursuitAction(driveTrain);
//        hitLever.setName("hitLever");
//        hitLever.setDependentActions(second6Intake);
//        hitLever.addPoint(LEVER_X, LEVER_Y, 90 * allianceSetup.getPolarity());
//        // move to intake
//        moveToLeverBalls.addPoint(LEVER_X, LEVER_Y + (220 * allianceSetup.getPolarity()), 90 * allianceSetup.getPolarity());
//        redAutoNear.addAction(hitLever);
//
//        PurePursuitAction moveToFarShoot2 = new PurePursuitAction(driveTrain);
//        moveToFarShoot2.setName("moveToFarShoot2");
//        moveToFarShoot2.setDependentActions(first3Intake);
//        moveToFarShoot2.addPoint(SHOOT_FAR_X, SHOOT_FAR_Y, 90 * allianceSetup.getPolarity());
//        redAutoNear.addAction(moveToFarShoot2);
//
//        ShootAllAction thirdShoot = new ShootAllAction(stopper, intake, shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO);
//        thirdShoot.setName("thirdShoot");
//        thirdShoot.setDependentActions(moveToFarShoot2);
//        redAutoNear.addAction(thirdShoot);
//
//        PurePursuitAction moveToFirstBalls = new PurePursuitAction(driveTrain);
//        moveToFirstBalls.setName("moveToFirstballs");
//        moveToFirstBalls.setDependentActions(thirdShoot);
//        moveToFirstBalls.addPoint(FIRST_BALL_X, FIRST_BALL_Y, 90 * allianceSetup.getPolarity());
//        // move to intake
//        moveToFirstBalls.addPoint(FIRST_BALL_X, FIRST_BALL_Y + (520 * allianceSetup.getPolarity()), 90 * allianceSetup.getPolarity());
//        redAutoNear.addAction(moveToFirstBalls);
//
//        IntakeFullAction third9Intake = new IntakeFullAction(intake);
//        third9Intake.setName("third9Intake");
//        third9Intake.setDependentActions(thirdShoot);
//        redAutoNear.addAction(third9Intake);
//
//        PurePursuitAction moveToFarShoot3 = new PurePursuitAction(driveTrain);
//        moveToFarShoot3.setName("moveToFarShoot3");
//        moveToFarShoot3.setDependentActions(third9Intake);
//        moveToFarShoot3.addPoint(SHOOT_FAR_X, SHOOT_FAR_Y, 90 * allianceSetup.getPolarity());
//        redAutoNear.addAction(moveToFarShoot3);
//
//        ShootAllAction fourthShoot = new ShootAllAction(stopper, intake, shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO);
//        fourthShoot.setName("fourthShoot");
//        fourthShoot.setDependentActions(moveToFarShoot2);
//        redAutoNear.addAction(fourthShoot);
//
//        PurePursuitAction moveToDepotBalls2 = new PurePursuitAction(driveTrain);
//        moveToDepotBalls2.setName("moveToDepotBalls2");
//        moveToDepotBalls2.setDependentActions(thirdShoot);
//        moveToDepotBalls2.addPoint(DEPOT_X, DEPOT_Y, 90 * allianceSetup.getPolarity());
//        // move to intake
//        moveToDepotBalls2.addPoint(DEPOT_X, DEPOT_Y + (520 * allianceSetup.getPolarity()), 90 * allianceSetup.getPolarity());
//        redAutoNear.addAction(moveToDepotBalls2);
//
//        IntakeFullAction fourth12Intake = new IntakeFullAction(intake);
//        fourth12Intake.setName("fourth12Intake");
//        fourth12Intake.setDependentActions(thirdShoot);
//        redAutoNear.addAction(fourth12Intake);
//
//        PurePursuitAction moveToFarShoot4 = new PurePursuitAction(driveTrain);
//        moveToFarShoot4.setName("moveToFarShoot4");
//        moveToFarShoot4.setDependentActions(fourth12Intake);
//        moveToFarShoot4.addPoint(SHOOT_FAR_X, SHOOT_FAR_Y, 90 * allianceSetup.getPolarity());
//        redAutoNear.addAction(moveToFarShoot4);
//
//        ShootAllAction fifthShoot = new ShootAllAction(stopper, intake, shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO);
//        fifthShoot.setName("fifthShoot");
//        fifthShoot.setDependentActions(moveToFarShoot4);
//        redAutoNear.addAction(fifthShoot);
//
//        PurePursuitAction park = new PurePursuitAction(driveTrain);
//        park.setName("park");
//        park.setDependentActions(fourthShoot);
//        park.addPoint(400, 400, 90);
//        park.setMaxCheckDoneCounter(20);
//        redAutoNear.addAction(park);

        waitForStart();
        while (opModeIsActive()) {
            redAutoNear.updateCheckDone();
            turretAutoAlign.updateCheckDone();
            KLog.d("Odometry", "Position: " + SharedData.getOdometryPosition());
        }
        cleanupRobot();
    }
}
