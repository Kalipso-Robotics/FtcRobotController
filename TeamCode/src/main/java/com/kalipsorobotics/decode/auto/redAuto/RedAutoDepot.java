package com.kalipsorobotics.decode.auto.redAuto;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.autoActions.pathActions.DepotRoundTrip;
import com.kalipsorobotics.actions.autoActions.pathActions.RoundTripAction;
import com.kalipsorobotics.actions.intake.IntakeStop;
import com.kalipsorobotics.actions.shooter.ShooterRun;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import com.kalipsorobotics.decode.configs.TurretConfig;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.DriveBrake;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.modules.shooter.ShooterRunMode;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KOpMode;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedAutoDepot")
public class RedAutoDepot extends KOpMode {
    KActionSet autoDepot;
    public final static double SHOOT_FAR_X = 150;
    public final static double SHOOT_FAR_Y = 100;
    Point farLaunchPoint =  new Point(SHOOT_FAR_X, SHOOT_FAR_Y);
    Point thirdLaunchPoint =  new Point(SHOOT_FAR_X, SHOOT_FAR_Y + 100);
    Point firstShootPoint = new Point(0,0);
    Point firstShotTargetPoint = new Point(Shooter.TARGET_POINT.getX() - 141.4213562373, Shooter.TARGET_POINT.getY() - 141.4213562373);
    private KActionSet lastTrip;
    private DriveTrain driveTrain;
    private DriveBrake driveBrake;
    Shooter shooter = null;
    Intake intake = null;
    Stopper stopper = null;
    Turret turret = null;
    DepotRoundTrip trip1 = null;
    TurretAutoAlign turretAutoAlign = null;

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.RED;
        SharedData.setAllianceColor(allianceColor);
        TurretConfig.TICKS_INIT_OFFSET = 0;
    }

    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        KLog.d("RedAutoDepot-Init", "Starting initializeRobot()");

        // Create your modules
        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        driveBrake = new DriveBrake(opModeUtilities);

        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000); // Optional: let hardware initialize

        // Create odometry
        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);

        autoDepot = new KActionSet();
        KLog.d("RedAutoDepot-Init", "Creating intake, shooter, stopper modules");
        KLog.d("RedAutoDepot-Init", "opModeUtilities is: " + (opModeUtilities != null ? "NOT NULL" : "NULL"));
        intake = new Intake(opModeUtilities);
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);
        shooterRun = new ShooterRun(opModeUtilities, shooter, 0, ShooterInterpolationConfig.MAX_HOOD);
        shooterRun.setShooterRunMode(ShooterRunMode.STOP);
        KLog.d("RedAutoDepot-Init", "Stopper created: " + (stopper != null ? "SUCCESS" : "NULL"));

        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);
        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, allianceColor);

        KLog.d("RedAutoDepot-Init", "Finished initializeRobot()");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        //No polarity here because multiplied externally


        // ----------------- FIRST SHOOT ----------------------

        RoundTripAction trip0 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, firstShotTargetPoint.multiplyY(allianceColor.getPolarity()), firstShootPoint, 0, false, true);
        trip0.setName("trip0");
        trip0.getMoveToBall().addPoint(0, 0, 0);
//        trip0.getPushBall().getRunUntilFullSpeed().setFullSpeedDurationMs(300);
        trip0.setShouldShooterStop(false);
        autoDepot.addAction(trip0);

        // ----------------- TRIP 1 (spike mark) ---------------------- ~5 sec
        trip1 = new DepotRoundTrip(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), farLaunchPoint.multiplyY(allianceColor.getPolarity()), 0, allianceColor);
        trip1.setName("trip1");
        trip1.setDependentActions(trip0);
//        trip1.getTrip().getPushBall().getRunUntilFullSpeed().setFullSpeedDurationMs(500);
        addPointsToTrip1SpikeMark();
        trip1.getTrip().setShouldShooterStop(false);
        autoDepot.addAction(trip1);

        // ----------------- TRIP 2 (corner) ---------------------- ~8 sec

        DepotRoundTrip trip2 = new DepotRoundTrip(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), thirdLaunchPoint.multiplyY(allianceColor.getPolarity()), 0, allianceColor);
        trip2.setName("trip2");
        trip2.setDependentActions(trip1);
        trip2.getTrip().getMoveToBall().clearPoints();
        // first try
//        trip2.getTrip().getMoveToBall().addPoint(200, 600 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity()); // x=15 -> corner
//        trip2.getTrip().getMoveToBall().addPoint(200, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip2.getTrip().getMoveToBall().addPoint(250,  1140 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());

        // retry
        trip2.getTrip().getMoveToBall().addPoint(15,  1140 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip2.getTrip().getMoveToBall().addPoint(SHOOT_FAR_X, (SHOOT_FAR_Y) * allianceColor.getPolarity() , 90 * allianceColor.getPolarity());
//        trip2.getTrip().getPushBall().getRunUntilFullSpeed().setFullSpeedDurationMs(500);
        trip2.getTrip().getMoveToBall().setFinalSearchRadius(150);
        trip2.getTrip().setShouldShooterStop(false);
        trip2.getTrip().getMoveToBall().setWithinRangeRadiusMM(150);
        trip2.getTrip().getMoveToBall().setPathAngleTolerance(45);
        trip2.getTrip().getMoveToBall().setFinalAngleLockingThresholdDegree(45);
        autoDepot.addAction(trip2);

        // ----------------- TRIP 3 (retry) ---------------------- ~5 sec

        DepotRoundTrip trip3 = generateRetryTrip(trip2, true);
        autoDepot.addAction(trip3);


        //-------------------TRIP 4 (retry) -------------------

        DepotRoundTrip trip4 = generateRetryTrip(trip3, false);
        autoDepot.addAction(trip4);


        //-------------------TRIP 5 (retry) -------------------

        DepotRoundTrip trip5 = generateRetryTrip(trip4, true);
        autoDepot.addAction(trip5);

        //-------------------TRIP 6 (retry) -------------------

        DepotRoundTrip trip6 = generateRetryTrip(trip5, false);
        lastTrip = trip6;
        autoDepot.addAction(trip6);

        // ----------------- PARK ----------------------

        shooterRun.setDependentActions(lastTrip);
        autoDepot.addAction(shooterRun);

        IntakeStop stopIntake = new IntakeStop(intake);
        stopIntake.setName("stopIntake");
        stopIntake.setDependentActions(lastTrip);
        autoDepot.addAction(stopIntake);

        PurePursuitAction park = new PurePursuitAction(driveTrain);
        park.setName("park");
        park.setDependentActions(lastTrip);
        park.addPoint(170, 540 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        park.setMaxCheckDoneCounter(20);
        autoDepot.addAction(park);
        KLog.d("auto", "-------------" +
                "-DEPOT AUTO STARTED-------------");
        KLog.d("RedAutoDepot-Run", "Before waitForStart() - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));
        waitForStart();
        KLog.d("RedAutoDepot-Run", "After waitForStart() - starting autonomous loop");
        while (opModeIsActive()) {
            autoDepot.updateCheckDone();
            turretAutoAlign.updateCheckDone();
            KLog.d("Odometry", "Position: " + SharedData.getOdometryWheelIMUPosition());
        }
        KLog.d("RedAutoDepot-Run", "Autonomous loop ended - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));
        KLog.d("Auto→TeleOp", "=== AUTO ENDING ===");
        KLog.d("Auto→TeleOp", "Final position: " + SharedData.getOdometryWheelIMUPosition());
        KLog.d("RedAutoDepot-Run", "Calling cleanupRobot()");
        cleanupRobot();
        KLog.d("Auto→TeleOp", "After cleanup position: " + SharedData.getOdometryWheelIMUPosition());
        KLog.d("RedAutoDepot-Run", "After cleanupRobot() - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));
    }


    // GO TO DEPOT FIRST ---------------
//    public void addPointsToTrip1() {
//
//        trip1.getTrip().getMoveToBall().clearPoints();
//        trip1.getTrip().getMoveToBall().addPoint(110, 600 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
//        trip1.getTrip().getMoveToBall().addPoint(110, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
//        trip1.getTrip().getMoveToBall().addPoint(110, 1110 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
//        trip1.getTrip().getMoveToBall().addPoint(15, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
//        trip1.getTrip().getMoveToBall().addPoint(15, 1118 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
//
//        trip1.getTrip().getMoveToBall().addPoint(SHOOT_FAR_X, SHOOT_FAR_Y * allianceColor.getPolarity() , 90 * allianceColor.getPolarity());
//    }


    // GO TO SPIKE MARK FIRST ---------------------
    public void addPointsToTrip1SpikeMark() {
        trip1.getTrip().getMoveToBall().clearPoints();
        trip1.getTrip().getMoveToBall().addPoint(650, 110 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getTrip().getMoveToBall().addPoint(650, 1140 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getTrip().getMoveToBall().addPoint(500, 1140 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getTrip().getMoveToBall().addPoint(SHOOT_FAR_X, SHOOT_FAR_Y * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
    }

    public DepotRoundTrip generateRetryTrip(DepotRoundTrip lastTrip, boolean sweepIn) {
        DepotRoundTrip retryTrip = new DepotRoundTrip(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), farLaunchPoint.multiplyY(allianceColor.getPolarity()), 0, allianceColor);
        retryTrip.getTrip().getMoveToBall().clearPoints();
        // first try
//        retryTrip.getTrip().getMoveToBall().addPoint(15, 600 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
//        retryTrip.getTrip().getMoveToBall().addPoint(15, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());

        if (sweepIn) {
            retryTrip.getTrip().getMoveToBall().addPoint(325, 1100 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
            // retry
            retryTrip.getTrip().getMoveToBall().addPoint(45, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
            retryTrip.getTrip().getMoveToBall().addPoint(45, 1100 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        } else {
            retryTrip.getTrip().getMoveToBall().addPoint(45, 1100 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
            retryTrip.getTrip().getMoveToBall().addPoint(325, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());

            // retry
            retryTrip.getTrip().getMoveToBall().addPoint(325, 1100 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        }

        retryTrip.getTrip().getMoveToBall().addPoint(SHOOT_FAR_X, (SHOOT_FAR_Y) * allianceColor.getPolarity() , 90 * allianceColor.getPolarity());
        retryTrip.getTrip().getMoveToBall().setFinalSearchRadius(150);
        retryTrip.getTrip().getMoveToBall().setWithinRangeRadiusMM(150);
        retryTrip.getTrip().getMoveToBall().setPathAngleTolerance(45);
        retryTrip.getTrip().getMoveToBall().setFinalAngleLockingThresholdDegree(45);
        retryTrip.getTrip().setShouldShooterStop(false);
        retryTrip.setName("retryTrip");
        retryTrip.setDependentActions(lastTrip);
        return retryTrip;
    }
}
