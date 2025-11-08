package com.kalipsorobotics.decode;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
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

import org.checkerframework.dataflow.qual.Pure;


@Autonomous(name = "RedAutoFarZone")
public class RedAutoFar extends KTeleOp {
    KActionSet redAutoNear;

    protected AllianceSetup allianceSetup = AllianceSetup.RED;

    protected final Point ROBOT_START_POINT_RED = Shooter.RED_TARGET_FROM_NEAR;
    final double DISTANCE_BETWEEN_BALLS = 610;
    final double DISTANCE_TO_BALL_FROM_ORIGIN = 1000;
    final double SHOOT_FAR_X = 100;
    final double SHOOT_FAR_Y = -40  * allianceSetup.getPolarity();
    final double LEVER_X = 1574.8;
    final double LEVER_Y = 1270  * allianceSetup.getPolarity();
    final double DEPOT_X = 0; // final ending 0 x
    final double DEPOT_Y = 1165  * allianceSetup.getPolarity(); // final ending 1,165.2 , 135degree heading
    final double FIRST_BALL_X = 660;
    final double FIRST_BALL_Y = DISTANCE_TO_BALL_FROM_ORIGIN  * allianceSetup.getPolarity();
    final double SECOND_BALL_X = FIRST_BALL_X + DISTANCE_BETWEEN_BALLS ;
    final double SECOND_BALL_Y = DISTANCE_TO_BALL_FROM_ORIGIN  * allianceSetup.getPolarity();
    final double THIRD_BALL_X = FIRST_BALL_X + DISTANCE_BETWEEN_BALLS*2;
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
        intakeFullAction = new IntakeFullAction(intake, 10);

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
//        ShootAllAction firstShoot = new ShootAllAction(stopper, intake, shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO);
//        firstShoot.setName("firstShoot");
//        redAutoNear.addAction(firstShoot);

        ShooterReady ready = new ShooterReady(shooter, Shooter.RED_TARGET_FROM_NEAR.multiplyY(allianceSetup.getPolarity()), LaunchPosition.NEAR); //TODO launch pos with polarity
        ready.setName("ready");
        redAutoNear.addAction(ready);

        PushBall shoot = new PushBall(stopper, intake);
        shoot.setName("shoot");
        shoot.setDependentActions(ready);
        redAutoNear.addAction(shoot);

//        PurePursuitAction moveToDepotBalls = new PurePursuitAction(driveTrain);
//        moveToDepotBalls.setName("moveToDepotBalls");
//        moveToDepotBalls.setDependentActions(firstShoot);
//        moveToDepotBalls.addPoint(DEPOT_X+300, DEPOT_Y-100, 155 * allianceSetup.getPolarity());
//        // move to intake
//        moveToDepotBalls.addPoint(DEPOT_X, DEPOT_Y, 155 * allianceSetup.getPolarity());
//        redAutoNear.addAction(moveToDepotBalls);

        PurePursuitAction moveToFirstBalls = new PurePursuitAction(driveTrain);
        moveToFirstBalls.addPoint(FIRST_BALL_X, FIRST_BALL_Y-660, 90 * allianceSetup.getPolarity());
        moveToFirstBalls.addPoint(FIRST_BALL_X, FIRST_BALL_Y, 90 * allianceSetup.getPolarity());
        moveToFirstBalls.addPoint(SHOOT_FAR_X, SHOOT_FAR_Y, 30 * allianceSetup.getPolarity()); // move to shoot
        moveToFirstBalls.setName("moveToFirstBalls");
        moveToFirstBalls.setDependentActions(shoot);
        redAutoNear.addAction(moveToFirstBalls);

        IntakeFullAction first3Intake = new IntakeFullAction(intake, 10);
        first3Intake.setName("first3Intake");
        first3Intake.setDependentActions(shoot);
        redAutoNear.addAction(first3Intake);

//        PurePursuitAction moveToFarShoot = new PurePursuitAction(driveTrain);
//        moveToFarShoot.setName("moveToFarShoot");
//        moveToFarShoot.setDependentActions(moveToFirstBalls);
//        moveToFarShoot.addPoint(SHOOT_FAR_X, SHOOT_FAR_Y, 30 * allianceSetup.getPolarity());
//        redAutoNear.addAction(moveToFarShoot);

//        ShootAllAction secondShoot = new ShootAllAction(stopper, intake, shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO);
//        secondShoot.setName("secondShoot");
//        secondShoot.setDependentActions(moveToFirstBalls);
//        redAutoNear.addAction(secondShoot);

        PushBall shoot2 = new PushBall(stopper, intake);
        shoot2.setName("shoot2");
        shoot2.setDependentActions(moveToFirstBalls);
        redAutoNear.addAction(shoot2);

        ShooterStop stop = new ShooterStop(shooter);
        stop.setName("stop");
        stop.setDependentActions(shoot2);
        redAutoNear.addAction(stop);

        WaitAction waitForShoot2 = new WaitAction(4000);
        waitForShoot2.setName("wait");
        waitForShoot2.setDependentActions(shoot2);
        redAutoNear.addAction(waitForShoot2);

        ShooterReady ready2 = new ShooterReady(shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO);
        ready2.setName("ready2");
        ready2.setDependentActions(waitForShoot2);
        redAutoNear.addAction(ready2);

        PurePursuitAction moveToLeverBalls = new PurePursuitAction(driveTrain);
        moveToLeverBalls.setName("moveToLeverBalls");
        moveToLeverBalls.setDependentActions(shoot2, moveToFirstBalls);
        moveToLeverBalls.addPoint(SECOND_BALL_X, (SECOND_BALL_Y-660) *allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        // move to intake
        moveToLeverBalls.addPoint(SECOND_BALL_X, SECOND_BALL_Y, 90 * allianceSetup.getPolarity());
        redAutoNear.addAction(moveToLeverBalls);

        IntakeFullAction second6Intake = new IntakeFullAction(intake, 20);
        second6Intake.setName("second6Intake");
        second6Intake.setDependentActions(shoot2);
        redAutoNear.addAction(second6Intake);

        PurePursuitAction hitLever = new PurePursuitAction(driveTrain);
        hitLever.setName("hitLever");
        hitLever.setDependentActions(second6Intake, moveToLeverBalls);
        hitLever.addPoint(LEVER_X -200, LEVER_Y -300, 90 * allianceSetup.getPolarity());
        // move to intake
        moveToLeverBalls.addPoint(LEVER_X, LEVER_Y, 50 * allianceSetup.getPolarity());
        redAutoNear.addAction(hitLever);

        PurePursuitAction moveToFarShoot2 = new PurePursuitAction(driveTrain);
        moveToFarShoot2.setName("moveToFarShoot2");
        moveToFarShoot2.setDependentActions(hitLever);
        moveToFarShoot2.addPoint(SHOOT_FAR_X, SHOOT_FAR_Y, 90 * allianceSetup.getPolarity());
        redAutoNear.addAction(moveToFarShoot2);

        PushBall shoot3 = new PushBall(stopper, intake);
        shoot3.setName("shoot3");
        shoot3.setDependentActions(moveToFarShoot2);
        redAutoNear.addAction(shoot3);


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
