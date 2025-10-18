package com.kalipsorobotics.decode;

import android.util.Log;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.intake.IntakeFullAction;
import com.kalipsorobotics.actions.intake.IntakeReverse;
import com.kalipsorobotics.actions.intake.IntakeRun;
import com.kalipsorobotics.actions.intake.IntakeStop;
import com.kalipsorobotics.actions.revolverActions.DetectColorsAction;
import com.kalipsorobotics.actions.revolverActions.FullShootMotifAction;
import com.kalipsorobotics.actions.revolverActions.RevolverTeleOp;
import com.kalipsorobotics.actions.shooter.KickBall;
import com.kalipsorobotics.actions.shooter.ShootAction;
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


@Autonomous(name="Red Auto Near")
public class RedAutoNear extends KTeleOp {
    KActionSet redAutoNear;

    protected AllianceSetup allianceSetup = AllianceSetup.RED;

    protected final Point ROBOT_START_POINT_RED = Shooter.RED_TARGET_FROM_NEAR;
    final double DISTANCE_BETWEEN_BALLS = 609.6;
    final double DISTANCE_TO_BALL_FROM_ORIGIN = 1066.8;
    final double DEPOT_X = 10;
    final double DEPOT_Y = 600;
    final double FIRST_BALL_X = 304.8;
    final double FIRST_BALL_Y = DISTANCE_TO_BALL_FROM_ORIGIN;
    final double SECOND_BALL_X = 304.8 + DISTANCE_BETWEEN_BALLS ;
    final double SECOND_BALL_Y = DISTANCE_TO_BALL_FROM_ORIGIN;
    final double THIRD_BALL_X = 304.8 + DISTANCE_BETWEEN_BALLS*2;
    final double THIRD_BALL_Y = DISTANCE_TO_BALL_FROM_ORIGIN;
    private DriveTrain driveTrain;
    TripleColorSensor colorSensors = null;
    Shooter shooter = null;
    Intake intake = null;
    ShooterReady shooterReady = null;
    ShooterStop shooterStop = null;
    ShootAction shootAction = null;

    LaunchPosition launchPosition = LaunchPosition.AUTO;
    KickBall kickBall = null;
    Revolver revolver = null;
    Turret turret = null;

    IntakeRun intakeRun = null;
    IntakeStop intakeStop = null;
    IntakeFullAction intakeFullAction = null;

    RevolverTeleOp revolverTeleOp = null;

    FullShootMotifAction fullShootMotifAction = null;

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
        revolver.getRevolverServo().setPosition(Revolver.REVOLVER_INDEX_0);

        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);

        intakeRun = new IntakeRun(intake);
        intakeStop = new IntakeStop(intake);
        intakeReverse = new IntakeReverse(intake);
        intakeFullAction = new IntakeFullAction(intake, revolver, colorSensors);

        revolverTeleOp = new RevolverTeleOp(revolver, false);

        turretAutoAlign = new TurretAutoAlign(turret, TurretAutoAlign.RED_X_INIT_SETUP, TurretAutoAlign.RED_Y_INIT_SETUP * allianceSetup.getPolarity());

        detectColorsAction = new DetectColorsAction(colorSensors, opModeUtilities);

        //todo just fed in testing motif pattern change later
        testingMotif = new MotifCamera.MotifPattern(MotifColor.PURPLE, MotifColor.PURPLE, MotifColor.GREEN);
//        testingMotif = new ObiliskDetection.MotifPattern(MotifColor.PURPLE, MotifColor.PURPLE, MotifColor.GREEN);
//        fullShootMotifAction = new FullShootMotifAction(revolver, shooter, testingMotif, colorSensors, opModeUtilities);

        shooterReady = new ShooterReady(shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO);
        shootAction = new ShootAction(shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO);
        shooterStop = new ShooterStop(shooter);
        kickBall = new KickBall(shooter);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        // auto code here
//        FullShootMotifAction firstShoot = new FullShootMotifAction(revolver, shooter, testingMotif, colorSensors, opModeUtilities);
//        firstShoot.setName("firstShoot");
//        redAutoNear.addAction(firstShoot);
//
        PurePursuitAction moveToDepotBalls = new PurePursuitAction(driveTrain);
        moveToDepotBalls.setName("moveToDepotBalls");
//        moveToDepotBalls.setDependentActions(firstShoot);
        moveToDepotBalls.addPoint(DEPOT_X, DEPOT_Y * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        moveToDepotBalls.addPoint(DEPOT_X + 200, (DEPOT_Y + 600) * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        redAutoNear.addAction(moveToDepotBalls);


        IntakeFullAction first3Intake = new IntakeFullAction(intake, revolver, colorSensors);
        first3Intake.setName("first3Intake");
        //first3Intake.setDependentActions(firstShoot);
        redAutoNear.addAction(first3Intake);
//

        PurePursuitAction moveToNearShoot = new PurePursuitAction(driveTrain);
        moveToNearShoot.setName("moveToNearShoot");
        moveToNearShoot.setDependentActions(moveToDepotBalls, first3Intake);
        moveToNearShoot.addPoint(0,0,0); // turret should align to goal not sure, also pos can be not 0,0 for faster
        redAutoNear.addAction(moveToNearShoot);
//
//        FullShootMotifAction secondShoot = new FullShootMotifAction(revolver, shooter, testingMotif, colorSensors, opModeUtilities);
//        secondShoot.setName("secondShoot");
//        secondShoot.setDependentActions(moveToNearShoot);
//        redAutoNear.addAction(secondShoot);
//
//        PurePursuitAction moveToFirstBalls = new PurePursuitAction(driveTrain);
//        moveToFirstBalls.setName("moveToFirstBalls");
//        moveToFirstBalls.setDependentActions(secondShoot);
//        moveToFirstBalls.addPoint(FIRST_BALL_X, FIRST_BALL_Y, 90);
//        redAutoNear.addAction(moveToFirstBalls);
//
//        IntakeCycle next6Intake = new IntakeCycle(driveTrain, intake, revolver, colorSensors, DEPOT_X, DEPOT_Y, 90);
//        next6Intake.setName("next6Intake");
//        next6Intake.setDependentActions(moveToFirstBalls);
//        redAutoNear.addAction(next6Intake);
//
//        PurePursuitAction moveToNearShoot2 = new PurePursuitAction(driveTrain);
//        moveToNearShoot2.setName("moveToNearShoot2");
//        moveToNearShoot2.setDependentActions(next6Intake);
//        moveToNearShoot2.addPoint(0,0,-15); // turret should align to goal not sure, also pos can be not 0,0 for faster
//        redAutoNear.addAction(moveToNearShoot2);
//
//        FullShootMotifAction thirdShoot = new FullShootMotifAction(revolver, shooter, testingMotif, colorSensors, opModeUtilities);
//        thirdShoot.setName("thirdShoot");
//        thirdShoot.setDependentActions(moveToNearShoot2);
//        redAutoNear.addAction(thirdShoot);
//
//        PurePursuitAction moveToSecondBalls = new PurePursuitAction(driveTrain);
//        moveToSecondBalls.setName("moveToSecondBalls");
//        moveToSecondBalls.setDependentActions(thirdShoot);
//        moveToSecondBalls.addPoint(SECOND_BALL_X, SECOND_BALL_Y, 90);
//        redAutoNear.addAction(moveToSecondBalls);
//
//        IntakeCycle final12Intake = new IntakeCycle(driveTrain, intake, revolver, colorSensors, FIRST_BALL_X, FIRST_BALL_Y, 90);
//        final12Intake.setName("final9Intake");
//        final12Intake.setDependentActions(moveToSecondBalls);
//        redAutoNear.addAction(final12Intake);
//
//        PurePursuitAction moveToNearShoot3 = new PurePursuitAction(driveTrain);
//        moveToNearShoot3.setName("moveToNearShoot3");
//        moveToNearShoot3.setDependentActions(final12Intake);
//        moveToNearShoot3.addPoint(0,0,-15); // turret should align to goal not sure, also pos can be not 0,0 for faster
//        redAutoNear.addAction(moveToNearShoot3);
//
//        FullShootMotifAction fourthShoot = new FullShootMotifAction(revolver, shooter, testingMotif, colorSensors, opModeUtilities);
//        fourthShoot.setName("fourthShoot");
//        fourthShoot.setDependentActions(moveToNearShoot3);
//        redAutoNear.addAction(fourthShoot);
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
