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
import com.kalipsorobotics.actions.shooter.ShooterRun;
import com.kalipsorobotics.actions.shooter.ShooterStop;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.actions.turret.TurretConfig;
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

@Autonomous(name = "RedAutoDepot")
public class AutoDepot extends KTeleOp {
    KActionSet autoDepot;

    protected AllianceSetup allianceSetup = AllianceSetup.RED;

    protected final Point ROBOT_START_POINT_RED = Shooter.RED_TARGET_FROM_FAR;
    final double SHOOT_FAR_X = 150;
    final double SHOOT_FAR_Y = -40;
    final double DEPOT_X = 150; // final ending 0 x
    final double DEPOT_Y = 1100; // final ending 1,165.2 , 135degree heading
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

        autoDepot = new KActionSet();
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
        Point farLaunchPoint =  new Point(SHOOT_FAR_X, SHOOT_FAR_Y * allianceSetup.getPolarity());
        Point firstShootPoint = new Point(0,0);

        // ----------------- FIRST SHOOT ----------------------

        RoundTripAction trip0 = new RoundTripAction(opModeUtilities, driveTrain, shooter, stopper, intake, Shooter.RED_TARGET_FROM_FAR.multiplyY(allianceSetup.getPolarity()), firstShootPoint, 0, false);
        trip0.setName("trip0");
        trip0.getMoveToBall().addPoint(firstShootPoint.getX(), firstShootPoint.getY()*allianceSetup.getPolarity(), 0);
        autoDepot.addAction(trip0);

        // ----------------- TRIP 1 ---------------------- ~5 sec

        RoundTripAction trip1 = new RoundTripAction(opModeUtilities, driveTrain, shooter, stopper, intake, Shooter.RED_TARGET_FROM_FAR.multiplyY(allianceSetup.getPolarity()), farLaunchPoint, 3000);
        trip1.setName("trip1");
        trip1.getMoveToBall().addPoint(DEPOT_X + 50, (DEPOT_Y - 300) * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        trip1.getMoveToBall().addPoint(DEPOT_X + 50, (DEPOT_Y-50) * allianceSetup.getPolarity() , 110 * allianceSetup.getPolarity());
        trip1.getMoveToBall().addPoint(DEPOT_X - 130, (DEPOT_Y-50) * allianceSetup.getPolarity() , 110 * allianceSetup.getPolarity());
        trip1.getMoveToBall().addPoint(DEPOT_X - 130, (DEPOT_Y-100) * allianceSetup.getPolarity() , 110 * allianceSetup.getPolarity());
        trip1.getMoveToBall().addPoint(DEPOT_X - 160, (DEPOT_Y-100) * allianceSetup.getPolarity() , 90 * allianceSetup.getPolarity());
        trip1.getMoveToBall().addPoint(DEPOT_X - 160, (DEPOT_Y-25) * allianceSetup.getPolarity() , 90 * allianceSetup.getPolarity());
        trip1.getMoveToBall().addPoint(SHOOT_FAR_X, SHOOT_FAR_Y * allianceSetup.getPolarity(), 30 * allianceSetup.getPolarity());
        trip1.setDependentActions(trip0);
        autoDepot.addAction(trip1);


        // ----------------- TRIP 2 ---------------------- ~8 sec

        RoundTripAction trip2 = new RoundTripAction(opModeUtilities, driveTrain, shooter, stopper, intake, Shooter.RED_TARGET_FROM_FAR.multiplyY(allianceSetup.getPolarity()),  farLaunchPoint, 6000);
        trip2.setName("trip2");
        trip2.getMoveToBall().addPoint((DEPOT_X + 150), (DEPOT_Y - 660) * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        trip2.getMoveToBall().addPoint((DEPOT_X + 150), (DEPOT_Y - 50) * allianceSetup.getPolarity() , 90 * allianceSetup.getPolarity());
        trip2.getMoveToBall().addPoint(SHOOT_FAR_X, SHOOT_FAR_Y * allianceSetup.getPolarity(), 30 * allianceSetup.getPolarity());
        trip2.setDependentActions(trip1);
        autoDepot.addAction(trip2);

        // ----------------- TRIP 3 ---------------------- ~5 sec

        RoundTripAction trip3 = new RoundTripAction(opModeUtilities, driveTrain, shooter, stopper, intake, Shooter.RED_TARGET_FROM_FAR.multiplyY(allianceSetup.getPolarity()), farLaunchPoint, 3000);
        trip3.setName("trip3");
        trip3.getMoveToBall().addPoint((DEPOT_X + 150), (DEPOT_Y - 660) * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        trip3.getMoveToBall().addPoint((DEPOT_X + 150), (DEPOT_Y - 50) * allianceSetup.getPolarity() , 90 * allianceSetup.getPolarity());
        trip3.getMoveToBall().addPoint(SHOOT_FAR_X, SHOOT_FAR_Y * allianceSetup.getPolarity(), 30 * allianceSetup.getPolarity());
        trip3.setDependentActions(trip2);
        autoDepot.addAction(trip3);

        // ----------------- PARK ----------------------

        PurePursuitAction park = new PurePursuitAction(driveTrain);
        park.setName("park");
        park.setDependentActions(trip3);
        park.addPoint(SHOOT_FAR_X + 400, SHOOT_FAR_Y * allianceSetup.getPolarity(), 90 * allianceSetup.getPolarity());
        park.setMaxCheckDoneCounter(20);
        autoDepot.addAction(park);

        waitForStart();
        while (opModeIsActive()) {
            autoDepot.updateCheckDone();
            turretAutoAlign.updateCheckDone();
            KLog.d("Odometry", "Position: " + SharedData.getOdometryPosition());
        }
        cleanupRobot();
    }
}
