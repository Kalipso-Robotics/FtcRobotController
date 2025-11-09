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


@Autonomous(name = "RedAutoFarZone")
public class RedAutoFar extends KTeleOp {
    KActionSet redAutoNear;

    protected AllianceSetup allianceSetup = AllianceSetup.RED;

    protected final Point ROBOT_START_POINT_RED = Shooter.RED_TARGET_FROM_NEAR;
    final double SHOOT_FAR_X = 150;
    final double SHOOT_FAR_Y = -40  * allianceSetup.getPolarity();
    final double LEVER_X = 1500;
    final double LEVER_Y = 1050  * allianceSetup.getPolarity();
    final double DEPOT_X = 200; // final ending 0 x
    final double DEPOT_Y = 1100  * allianceSetup.getPolarity(); // final ending 1,165.2 , 135degree heading
    final double FIRST_BALL_X = 680;
    final double FIRST_BALL_Y = 1050  * allianceSetup.getPolarity();
    final double SECOND_BALL_X = 1230;
    final double SECOND_BALL_Y = 1000  * allianceSetup.getPolarity();
    final double THIRD_BALL_X = 1830;
    final double THIRD_BALL_Y = 1000  * allianceSetup.getPolarity();
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
        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000); // Optional: let hardware initialize

        // Create odometry
        Odometry.setInstanceNull();
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
        intakeFullAction = new IntakeFullAction(stopper, intake, 10);

        revolverTeleOp = new RevolverTeleOp(revolver, false);

        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, TurretAutoAlign.RED_X_INIT_SETUP, TurretAutoAlign.RED_Y_INIT_SETUP * allianceSetup.getPolarity());

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

        // ----------------- FIRST SHOOT ----------------------

        ShooterReady ready = new ShooterReady(shooter, Shooter.RED_TARGET_FROM_NEAR.multiplyY(allianceSetup.getPolarity()), LaunchPosition.NEAR); //TODO launch pos with polarity
        ready.setName("ready");
        redAutoNear.addAction(ready);

        PushBall shoot = new PushBall(stopper, intake);
        shoot.setName("shoot");
        shoot.setDependentActions(ready);
        redAutoNear.addAction(shoot);

        // ----------------- TRIP 1 ----------------------

        PurePursuitAction moveToFirstBalls = new PurePursuitAction(driveTrain);
        moveToFirstBalls.addPoint(FIRST_BALL_X, FIRST_BALL_Y-660, 90 * allianceSetup.getPolarity());
        moveToFirstBalls.addPoint(FIRST_BALL_X, FIRST_BALL_Y, 90 * allianceSetup.getPolarity());
        moveToFirstBalls.addPoint(SHOOT_FAR_X, SHOOT_FAR_Y, 30 * allianceSetup.getPolarity()); // move to shoot
        moveToFirstBalls.setName("moveToFirstBalls");
        moveToFirstBalls.setDependentActions(shoot);
        redAutoNear.addAction(moveToFirstBalls);

        KLog.d("purepursuit", "moveToFirstBalls done");

        IntakeFullAction first3Intake = new IntakeFullAction(stopper, intake, 8);
        first3Intake.setName("first3Intake");
        first3Intake.setDependentActions(shoot);
        redAutoNear.addAction(first3Intake);

        PushBall shoot2 = new PushBall(stopper, intake);
        shoot2.setName("shoot2");
        shoot2.setDependentActions(moveToFirstBalls);
        redAutoNear.addAction(shoot2);

        ShooterStop stop = new ShooterStop(shooter);
        stop.setName("stop");
        stop.setDependentActions(shoot2);
        redAutoNear.addAction(stop);

        // ----------------- TRIP 2 ----------------------

        PurePursuitAction moveToLeverBalls = new PurePursuitAction(driveTrain);
        moveToLeverBalls.setName("moveToLeverBalls");
        moveToLeverBalls.setDependentActions(shoot2, moveToFirstBalls);
        moveToLeverBalls.addPoint(SECOND_BALL_X, (SECOND_BALL_Y-660) *allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        // move to intake
        moveToLeverBalls.addPoint(SECOND_BALL_X, SECOND_BALL_Y, 90 * allianceSetup.getPolarity());
        moveToLeverBalls.addPoint(LEVER_X -200, LEVER_Y -300, 90 * allianceSetup.getPolarity());
        // move to hit lever
        moveToLeverBalls.addPoint(LEVER_X, LEVER_Y-300, 0 * allianceSetup.getPolarity());
        moveToLeverBalls.addPoint(LEVER_X, LEVER_Y, 0 * allianceSetup.getPolarity());
        moveToLeverBalls.addPoint(SHOOT_FAR_X, SHOOT_FAR_Y, 45 * allianceSetup.getPolarity());
        moveToLeverBalls.setFinalSearchRadius(30);
        redAutoNear.addAction(moveToLeverBalls);

        WaitAction waitForShoot2 = new WaitAction(4000);
        waitForShoot2.setName("wait");
        waitForShoot2.setDependentActions(shoot2);
        redAutoNear.addAction(waitForShoot2);

        ShooterReady ready2 = new ShooterReady(shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.NEAR);
        ready2.setName("ready2");
        ready2.setDependentActions(waitForShoot2);
        redAutoNear.addAction(ready2);

        IntakeFullAction second6Intake = new IntakeFullAction(stopper, intake, 8);
        second6Intake.setName("second6Intake");
        second6Intake.setDependentActions(shoot2);
        redAutoNear.addAction(second6Intake);

        PushBall shoot3 = new PushBall(stopper, intake);
        shoot3.setName("shoot3");
        shoot3.setDependentActions(moveToLeverBalls, ready2);
        redAutoNear.addAction(shoot3);

        ShooterStop stop2 = new ShooterStop(shooter);
        stop2.setName("stop2");
        stop2.setDependentActions(shoot3);
        redAutoNear.addAction(stop2);

        // ----------------- TRIP 3 ----------------------

        PurePursuitAction moveToDepotBalls = new PurePursuitAction(driveTrain);
        moveToDepotBalls.setName("moveToDepotBalls");
        moveToDepotBalls.setDependentActions(shoot3);
        moveToDepotBalls.addPoint(DEPOT_X+300, DEPOT_Y, 145 * allianceSetup.getPolarity());
        // move to intake
        moveToDepotBalls.addPoint(DEPOT_X, DEPOT_Y, 145 * allianceSetup.getPolarity());
        moveToDepotBalls.addPoint(SHOOT_FAR_X, SHOOT_FAR_Y, 90 * allianceSetup.getPolarity());
        moveToLeverBalls.setFinalSearchRadius(30);
        redAutoNear.addAction(moveToDepotBalls);

        WaitAction waitForShoot3 = new WaitAction(4000);
        waitForShoot3.setName("wait");
        waitForShoot3.setDependentActions(shoot3);
        redAutoNear.addAction(waitForShoot3);

        ShooterReady ready3 = new ShooterReady(shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.NEAR);
        ready3.setName("ready2");
        ready3.setDependentActions(waitForShoot3);
        redAutoNear.addAction(ready3);

        IntakeFullAction third9Intake = new IntakeFullAction(stopper, intake, 8);
        third9Intake.setName("third9Intake");
        third9Intake.setDependentActions(shoot3);
        redAutoNear.addAction(third9Intake);

        PushBall shoot4 = new PushBall(stopper, intake);
        shoot4.setName("shoot4");
        shoot4.setDependentActions(moveToDepotBalls);
        redAutoNear.addAction(shoot4);

        ShooterStop stop3 = new ShooterStop(shooter);
        stop3.setName("stop2");
        stop3.setDependentActions(shoot4);
        redAutoNear.addAction(stop3);

        // ----------------- PARK ----------------------

        PurePursuitAction park = new PurePursuitAction(driveTrain);
        park.setName("park");
        park.setDependentActions(shoot4);
        park.addPoint(400, 400, 90);
        park.setMaxCheckDoneCounter(20);
        redAutoNear.addAction(park);

        waitForStart();
        while (opModeIsActive()) {
            redAutoNear.updateCheckDone();
            turretAutoAlign.updateCheckDone();
            KLog.d("Odometry", "Position: " + SharedData.getOdometryPosition());
        }
        cleanupRobot();
    }
}
