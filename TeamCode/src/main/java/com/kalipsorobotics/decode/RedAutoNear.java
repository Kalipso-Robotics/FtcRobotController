package com.kalipsorobotics.decode;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.intake.IntakeFullAction;
import com.kalipsorobotics.actions.intake.IntakeReverse;
import com.kalipsorobotics.actions.intake.IntakeRun;
import com.kalipsorobotics.actions.intake.IntakeStop;
import com.kalipsorobotics.actions.revolverActions.DetectColorsAction;
import com.kalipsorobotics.actions.revolverActions.RevolverTeleOp;
import com.kalipsorobotics.actions.shooter.ShootAllAction;
import com.kalipsorobotics.actions.shooter.ShooterReady;
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
import com.kalipsorobotics.modules.Revolver;
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

    final double SHOOT_NEAR_X = 2400;
    final double SHOOT_NEAR_Y = 300;
    final double LEVER_X = 1500;
    final double LEVER_Y = 1060;
    final double DEPOT_X = 150; // final ending 0 x
    final double DEPOT_Y = 1100  ; // final ending 1,165.2 , 135degree heading
    final double FIRST_BALL_X = 680;
    final double FIRST_BALL_Y = 1050;
    final double SECOND_BALL_X = 1230;
    final double SECOND_BALL_Y = 1050;
    final double THIRD_BALL_X = 1830;
    final double THIRD_BALL_Y = 1000;
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
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, 3028.98, 746.18, -2.4137); //3015.93, 765.86, -2.4030
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

        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, TurretConfig.RED_X_INIT_SETUP, TurretConfig.RED_Y_INIT_SETUP * allianceSetup.getPolarity());

        detectColorsAction = new DetectColorsAction(colorSensors, opModeUtilities);

        //todo just fed in testing motif pattern change later
        testingMotif = new MotifCamera.MotifPattern(MotifColor.PURPLE, MotifColor.PURPLE, MotifColor.GREEN);
//        testingMotif = new ObiliskDetection.MotifPattern(MotifColor.PURPLE, MotifColor.PURPLE, MotifColor.GREEN);
//        fullShootMotifAction = new FullShootMotifAction(revolver, shooter, testingMotif, colorSensors, opModeUtilities);

        shooterReady = new ShooterReady(shooter, Shooter.RED_TARGET_FROM_FAR, LaunchPosition.AUTO);
        shootAction = new ShootAllAction(stopper, intake, shooter, Shooter.RED_TARGET_FROM_FAR);
        shooterStop = new ShooterStop(shooter);
        pushBall = new PushBall(stopper, intake, shooter);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        Point nearLaunchPoint =  new Point(SHOOT_NEAR_X, SHOOT_NEAR_Y * allianceSetup.getPolarity());

        // ----------------- FIRST SHOOT ----------------------
        ShooterReady ready = new ShooterReady(shooter, Shooter.RED_TARGET_FROM_FAR.multiplyY(allianceSetup.getPolarity()), LaunchPosition.NEAR_INIT); //TODO launch pos with polarity
        ready.setName("ready");
        redAutoNear.addAction(ready);

        PushBall shoot = new PushBall(stopper, intake, shooter);
        shoot.setName("shoot");
        shoot.setDependentActions(ready);
        redAutoNear.addAction(shoot);

        ShooterStop stop = new ShooterStop(shooter);
        stop.setName("stop");
        stop.setDependentActions(shoot);
        redAutoNear.addAction(stop);

        // ----------------- TRIP 1 ----------------------
        RoundTripAction trip1 = new RoundTripAction(opModeUtilities, driveTrain, shooter, stopper, intake, Shooter.RED_TARGET_FROM_FAR.multiplyY(allianceSetup.getPolarity()), nearLaunchPoint, 4000);
        trip1.setName("trip1");
        trip1.getMoveToBall().addPoint(THIRD_BALL_X, (THIRD_BALL_Y - 660) * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        trip1.getMoveToBall().addPoint(THIRD_BALL_X, THIRD_BALL_Y * allianceSetup.getPolarity() , 90 * allianceSetup.getPolarity());
        // move to hit lever
        trip1.getMoveToBall().addPoint(LEVER_X, (LEVER_Y - 300) * allianceSetup.getPolarity(), 0);
        trip1.getMoveToBall().addPoint(LEVER_X, LEVER_Y * allianceSetup.getPolarity(), 0);
        trip1.getMoveToBall().addPoint(SHOOT_NEAR_X, SHOOT_NEAR_Y * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        trip1.setDependentActions(shoot);
        redAutoNear.addAction(trip1);

        // ----------------- TRIP 2 ----------------------

        RoundTripAction trip2 = new RoundTripAction(opModeUtilities, driveTrain, shooter, stopper, intake, Shooter.RED_TARGET_FROM_FAR.multiplyY(allianceSetup.getPolarity()), nearLaunchPoint, 3500);
        trip2.setName("trip3");
        trip2.getMoveToBall().addPoint(SECOND_BALL_X, (SECOND_BALL_Y - 660) * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        // move to intake
        trip2.getMoveToBall().addPoint(SECOND_BALL_X, SECOND_BALL_Y * allianceSetup.getPolarity() , 90 * allianceSetup.getPolarity());
        trip2.getMoveToBall().addPoint(SHOOT_NEAR_X, SHOOT_NEAR_Y * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        trip2.setDependentActions(trip1);
        redAutoNear.addAction(trip2);


        // ----------------- TRIP 3 ----------------------

        RoundTripAction trip3 = new RoundTripAction(opModeUtilities, driveTrain, shooter, stopper, intake, Shooter.RED_TARGET_FROM_FAR.multiplyY(allianceSetup.getPolarity()), nearLaunchPoint, 3500);
        trip3.setName("trip3");
        trip3.getMoveToBall().addPoint(FIRST_BALL_X, (FIRST_BALL_Y - 660) * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        // move to intake
        trip3.getMoveToBall().addPoint(FIRST_BALL_X, FIRST_BALL_Y * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        trip3.getMoveToBall().addPoint(SHOOT_NEAR_X, SHOOT_NEAR_Y * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        trip3.setDependentActions(trip2);
        redAutoNear.addAction(trip3);

        // ----------------- PARK ----------------------

        PurePursuitAction park = new PurePursuitAction(driveTrain);
        park.setName("park");
        park.setDependentActions(trip3);
        park.addPoint(FIRST_BALL_X, (FIRST_BALL_Y - 660) * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
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
