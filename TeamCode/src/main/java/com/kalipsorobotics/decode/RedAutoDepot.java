package com.kalipsorobotics.decode;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.autoActions.pathActions.DepotRoundTrip;
import com.kalipsorobotics.actions.autoActions.pathActions.RoundTripAction;
import com.kalipsorobotics.actions.intake.IntakeStop;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.DriveBrake;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.modules.shooter.Shooter;
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
        KLog.d("RedAutoDepot-Init", "Stopper created: " + (stopper != null ? "SUCCESS" : "NULL"));

        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);
        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, allianceColor);

        KLog.d("RedAutoDepot-Init", "Finished initializeRobot()");
    }

    public void addPointsToTrip1() {

        trip1.getTrip().getMoveToBall().clearPoints();
        trip1.getTrip().getMoveToBall().addPoint(110, 600 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getTrip().getMoveToBall().addPoint(110, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getTrip().getMoveToBall().addPoint(110, 1110 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getTrip().getMoveToBall().addPoint(15, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getTrip().getMoveToBall().addPoint(15, 1118 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());

        trip1.getTrip().getMoveToBall().addPoint(SHOOT_FAR_X, SHOOT_FAR_Y * allianceColor.getPolarity() , 90 * allianceColor.getPolarity());
    }



    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        //No polarity here because multiplied externally
        Point farLaunchPoint =  new Point(SHOOT_FAR_X, SHOOT_FAR_Y);
        Point thirdLaunchPoint =  new Point(SHOOT_FAR_X, SHOOT_FAR_Y + 100);
        Point firstShootPoint = new Point(0,0);

        // ----------------- FIRST SHOOT ----------------------

        RoundTripAction trip0 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), firstShootPoint, 0, false);
        trip0.setName("trip0");
        trip0.getMoveToBall().addPoint(0, 0, 0);
        autoDepot.addAction(trip0);

        // ----------------- TRIP 1 ---------------------- ~5 sec

        trip1 = new DepotRoundTrip(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), farLaunchPoint.multiplyY(allianceColor.getPolarity()), 2000, allianceColor);
        trip1.setName("trip1");
        trip1.setDependentActions(trip0);
        addPointsToTrip1();
        autoDepot.addAction(trip1);

        // ----------------- TRIP 2 ---------------------- ~8 sec

        DepotRoundTrip trip2 = new DepotRoundTrip(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), farLaunchPoint.multiplyY(allianceColor.getPolarity()), 2000, allianceColor);
        trip2.setName("trip2");
        trip2.setDependentActions(trip1);
        autoDepot.addAction(trip2);

        // ----------------- TRIP 3 ---------------------- ~5 sec

        DepotRoundTrip trip3 = new DepotRoundTrip(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), thirdLaunchPoint.multiplyY(allianceColor.getPolarity()), 2000, allianceColor);
        trip3.setName("trip3");
        trip3.setDependentActions(trip2);
        trip3.getTrip().getMoveToBall().clearPoints();
        trip3.getTrip().getMoveToBall().addPoint(15, 600 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip3.getTrip().getMoveToBall().addPoint(15, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip3.getTrip().getMoveToBall().addPoint(15,  1168 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip3.getTrip().getMoveToBall().addPoint(223, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip3.getTrip().getMoveToBall().addPoint(223,  1168 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip3.getTrip().getMoveToBall().addPoint(SHOOT_FAR_X, (SHOOT_FAR_Y) * allianceColor.getPolarity() , 90 * allianceColor.getPolarity());
        autoDepot.addAction(trip3);

        //-------------------TRIP 4 ------------------- ~ 19qp01
//
//        DepotRoundTrip trip4 = new DepotRoundTrip(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), farLaunchPoint.multiplyY(allianceColor.getPolarity()), 2000, allianceColor);
//        trip4.setName("trip4");
//        trip4.setDependentActions(trip3);
//        autoDepot.addAction(trip4);

        // ----------------- PARK ----------------------

        IntakeStop stopIntake = new IntakeStop(intake);
        stopIntake.setName("stopIntake");
        stopIntake.setDependentActions(trip3);
        autoDepot.addAction(stopIntake);

        PurePursuitAction park = new PurePursuitAction(driveTrain);
        park.setName("park");
        park.setDependentActions(trip3);
        park.addPoint(170, 540 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        park.setMaxCheckDoneCounter(20);
        autoDepot.addAction(park);
        KLog.d("auto", "--------------DEPOT AUTO STARTED-------------");
        KLog.d("RedAutoDepot-Run", "Before waitForStart() - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));
        waitForStart();
        KLog.d("RedAutoDepot-Run", "After waitForStart() - starting autonomous loop");
        while (opModeIsActive()) {
            autoDepot.updateCheckDone();
            turretAutoAlign.updateCheckDone();
            KLog.d("Odometry", "Position: " + SharedData.getOdometryIMUPosition());
        }
        KLog.d("RedAutoDepot-Run", "Autonomous loop ended - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));
        KLog.d("Auto→TeleOp", "=== AUTO ENDING ===");
        KLog.d("Auto→TeleOp", "Final position: " + SharedData.getOdometryIMUPosition());
        KLog.d("RedAutoDepot-Run", "Calling cleanupRobot()");
        cleanupRobot();
        KLog.d("Auto→TeleOp", "After cleanup position: " + SharedData.getOdometryIMUPosition());
        KLog.d("RedAutoDepot-Run", "After cleanupRobot() - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));
    }


}
