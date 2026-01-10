package com.kalipsorobotics.decode.auto.redAuto;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.autoActions.pathActions.DepotRoundTrip;
import com.kalipsorobotics.actions.autoActions.pathActions.RampCycleAction;
import com.kalipsorobotics.actions.autoActions.pathActions.RoundTripAction;
import com.kalipsorobotics.actions.intake.IntakeStop;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.Point;
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

@Autonomous(name="RedAutoDepotRampCycle")
public class RedAutoDepotRampCycle extends KOpMode {
    KActionSet redAutoDepot;
    public final static double SHOOT_FAR_X = 150;
    public final static double SHOOT_FAR_Y = 100;

    public DriveTrain driveTrain;
    Shooter shooter = null;
    Intake intake = null;
    Stopper stopper = null;
    Turret turret = null;
    TurretAutoAlign turretAutoAlign = null;

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.RED;
        SharedData.setAllianceColor(allianceColor);
    }

    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        // Create your modules
        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000); // Optional: let hardware initialize

        // Create odometry
        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);


        redAutoDepot = new KActionSet();
        intake = new Intake(opModeUtilities);
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);

        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);
        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, allianceColor);

        turretAutoAlign.setToleranceDeg(1.5);
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
        redAutoDepot.addAction(trip0);

        // ----------------- TRIP 1 ---------------------- ~5 sec

        DepotRoundTrip trip1 = new DepotRoundTrip(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), farLaunchPoint.multiplyY(allianceColor.getPolarity()), 2000.0, allianceColor);
        trip1.setName("trip1");
        trip1.setDependentActions(trip0);
        trip1.getTrip().getMoveToBall().clearPoints();
        trip1.getTrip().getMoveToBall().addPoint(727, 110 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getTrip().getMoveToBall().addPoint(727, 1150 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getTrip().getMoveToBall().addPoint(SHOOT_FAR_X, SHOOT_FAR_Y * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        redAutoDepot.addAction(trip1);

        // ----------------- TRIP 2 ---------------------- ~8 sec

        RampCycleAction trip2 = new RampCycleAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, allianceColor, farLaunchPoint, 90);
        trip2.setName("trip2");
        trip2.setDependentActions(trip1);
        redAutoDepot.addAction(trip2);

        // ----------------- TRIP 3 ---------------------- ~5 sec

        RampCycleAction trip3 = new RampCycleAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, allianceColor, farLaunchPoint, 90);
        trip3.setName("trip3");
        trip3.setDependentActions(trip2);
        redAutoDepot.addAction(trip3);

        IntakeStop stopIntake = new IntakeStop(intake);
        stopIntake.setName("stopIntake");
        stopIntake.setDependentActions(trip3);
        redAutoDepot.addAction(stopIntake);

        PurePursuitAction park = new PurePursuitAction(driveTrain);
        park.setName("park");
        park.setDependentActions(trip3);
        park.addPoint(170, 540 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        park.setMaxCheckDoneCounter(20);
        redAutoDepot.addAction(park);
        KLog.d("auto", "--------------DEPOT AUTO STARTED-------------");
        KLog.d("BlueAutoDepot-Run", "Before waitForStart() - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));
        waitForStart();
        KLog.d("BlueAutoDepot-Run", "After waitForStart() - starting autonomous loop");
        while (opModeIsActive()) {
            redAutoDepot.updateCheckDone();
            turretAutoAlign.updateCheckDone();
            KLog.d("Odometry", "Position: " + SharedData.getOdometryWheelIMUPosition());
        }
        KLog.d("BlueAutoDepot-Run", "Autonomous loop ended - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));
        KLog.d("Auto→TeleOp", "=== AUTO ENDING ===");
        KLog.d("Auto→TeleOp", "Final position: " + SharedData.getOdometryWheelIMUPosition());
        KLog.d("BlueAutoDepot-Run", "Calling cleanupRobot()");
        cleanupRobot();
        KLog.d("Auto→TeleOp", "After cleanup position: " + SharedData.getOdometryWheelIMUPosition());
        KLog.d("blueAutoDepot-Run", "After cleanupRobot() - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));
    }
}
