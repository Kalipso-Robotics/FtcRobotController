package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions.DepotRoundTrip;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions.RoundTripAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.cameraVision.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveBrake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.ShooterRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

enum DepotTrips {
    SWEEP_IN,
    SWEEP_OUT,
    INTAKE_IN,
    SWEEP_TUNNEL,
    SWEEP_0,
    SWEEP_400,
    SWEEP_800
}

@Autonomous
public class RedAutoDepot extends KOpMode {
    KActionSet autoDepot;
    public final static double SHOOT_FAR_X = 150;
    public final static double SHOOT_FAR_Y = 100;
    Point farLaunchPoint =  new Point(SHOOT_FAR_X, SHOOT_FAR_Y);
    Point thirdLaunchPoint =  new Point(SHOOT_FAR_X, SHOOT_FAR_Y + 100);
    Point firstShootPoint = new Point(0,0);
    Point firstShotTargetPoint = new Point(Shooter.TARGET_POINT.getX(), Shooter.TARGET_POINT.getY() + 141.4213562373);
    private KActionSet lastTrip;
    private DriveTrain driveTrain;
    private DriveBrake driveBrake;
    Shooter shooter = null;
    Intake intake = null;
    Stopper stopper = null;
    Turret turret = null;
    DepotRoundTrip trip1 = null;
    RoundTripAction trip0 = null;
    DepotRoundTrip trip2 = null;
    DepotRoundTrip trip3 = null;
    DepotRoundTrip trip4 = null;
    DepotRoundTrip trip5 = null;
    DepotRoundTrip trip6 = null;


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
        OpModeUtilities.runOdometryExecutorService(odoExecutorService, odometry);

        autoDepot = new KActionSet();
        KLog.d("RedAutoDepot-Init", "Creating intake, shooter, stopper modules");
        KLog.d("RedAutoDepot-Init", () -> "opModeUtilities is: " + (opModeUtilities != null ? "NOT NULL" : "NULL"));
        intake = new Intake(opModeUtilities);
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);
        shooterRun = new ShooterRun(opModeUtilities, shooter, 0, ShooterInterpolationConfig.MAX_HOOD);
        shooterRun.setShooterRunMode(ShooterRunMode.STOP);
        KLog.d("RedAutoDepot-Init", () -> "Stopper created: " + (stopper != null ? "SUCCESS" : "NULL"));

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

        trip0 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, firstShotTargetPoint.multiplyY(allianceColor.getPolarity()), firstShootPoint, 0, false, true);
        trip0.setName("trip0");
        trip0.getShooterReady().setName("ShooterReady_trip0");
        trip0.getMoveToBall().addPoint(0, 0, 0);
//        trip0.getPushBall().getRunUntilFullSpeed().setFullSpeedDurationMs(300);
        trip0.setShouldShooterStop(false);
        autoDepot.addAction(trip0);

        // ----------------- TRIP 1 (spike mark) ---------------------- ~5 sec

        trip1 = new DepotRoundTrip(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), farLaunchPoint.multiplyY(allianceColor.getPolarity()), 0, allianceColor);
        trip1.setName("trip1");
        trip1.getTrip().getShooterReady().setName("ShooterReady_trip1");
        trip1.setDependentActions(trip0);
//        trip1.getTrip().getPushBall().getRunUntilFullSpeed().setFullSpeedDurationMs(500);
        addPointsToTrip1SpikeMark();
        trip1.getTrip().setShouldShooterStop(false);
        autoDepot.addAction(trip1);

        // ----------------- TRIP 2 (corner) ---------------------- ~8 sec

        trip2 = new DepotRoundTrip(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), thirdLaunchPoint.multiplyY(allianceColor.getPolarity()), 0, allianceColor);
        trip2.setName("trip2");
        trip2.getTrip().getShooterReady().setName("ShooterReady_trip2");
        trip2.setDependentActions(trip1);
        trip2.getTrip().getMoveToBall().clearPoints();
        // first try
//        trip2.getTrip().getMoveToBall().addPoint(200, 600 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity()); // x=15 -> corner
//        trip2.getTrip().getMoveToBall().addPoint(200, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip2.getTrip().getMoveToBall().addPoint(0,  1140 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());

        // retry
        trip2.getTrip().getMoveToBall().addPoint(250,  1160 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip2.getTrip().getMoveToBall().addPoint(SHOOT_FAR_X, (SHOOT_FAR_Y) * allianceColor.getPolarity() , 90 * allianceColor.getPolarity());
//        trip2.getTrip().getPushBall().getRunUntilFullSpeed().setFullSpeedDurationMs(500);
        trip2.getTrip().getMoveToBall().setFinalSearchRadiusMM(150);
        trip2.getTrip().setShouldShooterStop(false);
        trip2.getTrip().getPurePursuitReadyShooting().setDistanceThresholdMM(150);
        trip2.getTrip().getMoveToBall().setPathAngleToleranceDeg(45);
        trip2.getTrip().getMoveToBall().setFinalAngleLockingThresholdDeg(45);
        autoDepot.addAction(trip2);

        // ----------------- TRIP 3 (retry) ---------------------- ~5 sec

        handleTrip3();

        //-------------------TRIP 4 (retry) -------------------

        handleTrip4();

        //-------------------TRIP 5 (retry) -------------------

        handleTrip5();

        //-------------------TRIP 6 (retry) -------------------

        trip6 = handleTrip6(trip5);
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
        KLog.d("RedAutoDepot-Run", () -> "Before waitForStart() - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));
        stopper.setPosition(ModuleConfig.STOPPER_SERVO_CLOSED_POS);
        waitForStart();
        KLog.d("RedAutoDepot-Run", "After waitForStart() - starting autonomous loop");
        while (opModeIsActive()) {
            autoDepot.updateCheckDone();
            turretAutoAlign.updateCheckDone();
            KLog.d("Odometry", () -> "Position: " + SharedData.getOdometryWheelIMUPosition());
        }
        KLog.d("RedAutoDepot-Run", () -> "Autonomous loop ended - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));
        KLog.d("Auto→TeleOp", "=== AUTO ENDING ===");
        KLog.d("Auto→TeleOp", () -> "Final position: " + SharedData.getOdometryWheelIMUPosition());
        KLog.d("RedAutoDepot-Run", "Calling cleanupRobot()");
        cleanupRobot();
        KLog.d("Auto→TeleOp", () -> "After cleanup position: " + SharedData.getOdometryWheelIMUPosition());
        KLog.d("RedAutoDepot-Run", () -> "After cleanupRobot() - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));
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


    protected void handleTrip3() {
        trip3 = generateRetryTrip(trip2, DepotTrips.SWEEP_IN);
        autoDepot.addAction(trip3);
    }

    protected void handleTrip4() {
        trip4 = generateRetryTrip(trip3, DepotTrips.SWEEP_OUT);
        autoDepot.addAction(trip4);
    }

    protected void handleTrip5() {
        trip5 = generateRetryTrip(trip4, DepotTrips.SWEEP_IN);
        autoDepot.addAction(trip5);
    }

    protected DepotRoundTrip handleTrip6(DepotRoundTrip trip5) {
        return generateRetryTrip(trip5, DepotTrips.SWEEP_800);
    }


    // GO TO SPIKE MARK FIRST ---------------------
    public void addPointsToTrip1SpikeMark() {
        trip1.getTrip().getMoveToBall().clearPoints();
        trip1.getTrip().getMoveToBall().addPoint(650, 110 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getTrip().getMoveToBall().addPoint(650, 1100 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getTrip().getMoveToBall().addPoint(500, 1100 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getTrip().getMoveToBall().addPoint(SHOOT_FAR_X, SHOOT_FAR_Y * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
    }

    public DepotRoundTrip generateRetryTrip(DepotRoundTrip lastTrip, DepotTrips trip) {
        DepotRoundTrip retryTrip = new DepotRoundTrip(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), farLaunchPoint.multiplyY(allianceColor.getPolarity()), 0, allianceColor);
        retryTrip.getTrip().getShooterReady().setName("ShooterReady_retryTrip");
        retryTrip.getTrip().getMoveToBall().clearPoints();
        // first try
//        retryTrip.getTrip().getMoveToBall().addPoint(15, 600 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
//        retryTrip.getTrip().getMoveToBall().addPoint(15, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        switch (trip) {
            case SWEEP_IN:
                retryTrip.getTrip().getMoveToBall().addPoint(325, 1075 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
                // retry
                retryTrip.getTrip().getMoveToBall().addPoint(45, 800 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
                retryTrip.getTrip().getMoveToBall().addPoint(45, 1075 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
                break;
            case SWEEP_OUT:
                retryTrip.getTrip().getMoveToBall().addPoint(25, 1075 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
                retryTrip.getTrip().getMoveToBall().addPoint(325, 800 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
                // retry
                retryTrip.getTrip().getMoveToBall().addPoint(325, 1075 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
                break;
            case INTAKE_IN:
                retryTrip.getTrip().getMoveToBall().addPoint(10, 1075 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
                break;
            case SWEEP_TUNNEL:
                retryTrip.getTrip().getMoveToBall().addPoint(25, 1075 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
                retryTrip.getTrip().getMoveToBall().addPoint(25, 700 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
                retryTrip.getTrip().getMoveToBall().addPoint(850, 1075 * allianceColor.getPolarity(), 25 * allianceColor.getPolarity());
                break;
            case SWEEP_0:
                retryTrip.getTrip().getMoveToBall().addPoint(0  , 1075 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
                break;
            case SWEEP_400:
                retryTrip.getTrip().getMoveToBall().addPoint(400  , 1075 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
                break;
            case SWEEP_800:
                retryTrip.getTrip().getMoveToBall().addPoint(800  , 1075 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
                break;
        }

        retryTrip.getTrip().getMoveToBall().addPoint(SHOOT_FAR_X, (SHOOT_FAR_Y) * allianceColor.getPolarity() , 90 * allianceColor.getPolarity());
        retryTrip.getTrip().getMoveToBall().setFinalSearchRadiusMM(150);
        retryTrip.getTrip().getPurePursuitReadyShooting().setDistanceThresholdMM(150);
        retryTrip.getTrip().getMoveToBall().setPathAngleToleranceDeg(45);
        retryTrip.getTrip().getMoveToBall().setFinalAngleLockingThresholdDeg(45);
        retryTrip.getTrip().setShouldShooterStop(false);
        retryTrip.setName("depotTrip");
        retryTrip.setDependentActions(lastTrip);
        return retryTrip;
    }
}
