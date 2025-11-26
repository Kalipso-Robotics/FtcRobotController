package com.kalipsorobotics.decode;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.autoActions.pathActions.DepotRoundTrip;
import com.kalipsorobotics.actions.autoActions.pathActions.RoundTripAction;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.actions.turret.TurretConfig;
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

@Autonomous(name = "RedAutoDepot")
public class RedAutoDepot extends KOpMode {
    KActionSet autoDepot;

    public final static double SHOOT_FAR_X = 30;
    public final static double SHOOT_FAR_Y = 89;
    private DriveTrain driveTrain;
    Shooter shooter = null;
    Intake intake = null;
    Stopper stopper = null;
    Turret turret = null;

    TurretAutoAlign turretAutoAlign = null;

    @Override
    protected void initializeRobot() {
        this.allianceColor = AllianceColor.RED;
        SharedData.setAllianceColor(allianceColor);
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

        autoDepot = new KActionSet();
        intake = new Intake(opModeUtilities);
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);

        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);
        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, allianceColor);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        //No polarity here because multiplied externally
        Point farLaunchPoint =  new Point(SHOOT_FAR_X, SHOOT_FAR_Y);
        Point firstShootPoint = new Point(0,0);

        // ----------------- FIRST SHOOT ----------------------

        RoundTripAction trip0 = new RoundTripAction(opModeUtilities, driveTrain, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), firstShootPoint, 0, false);
        trip0.setName("trip0");
        trip0.getMoveToBall().addPoint(0, 0, 0);
        autoDepot.addAction(trip0);

        // ----------------- TRIP 1 ---------------------- ~5 sec

        DepotRoundTrip trip1 = new DepotRoundTrip(opModeUtilities, driveTrain, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), farLaunchPoint.multiplyY(allianceColor.getPolarity()), 2000, allianceColor);
        trip1.setName("trip1");
        trip1.setDependentActions(trip0);
        trip1.getTrip().getMoveToBall().clearPoint();
        trip1.getTrip().getMoveToBall().addPoint(SHOOT_FAR_X, 600 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getTrip().getMoveToBall().addPoint(SHOOT_FAR_X, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getTrip().getMoveToBall().addPoint(SHOOT_FAR_X, 1168 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getTrip().getMoveToBall().addPoint(SHOOT_FAR_X, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getTrip().getMoveToBall().addPoint(SHOOT_FAR_X, 1168 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());

        trip1.getTrip().getMoveToBall().addPoint(SHOOT_FAR_X, SHOOT_FAR_Y * allianceColor.getPolarity() , 90 * allianceColor.getPolarity());

        autoDepot.addAction(trip1);


        // ----------------- TRIP 2 ---------------------- ~8 sec

        DepotRoundTrip trip2 = new DepotRoundTrip(opModeUtilities, driveTrain, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), farLaunchPoint.multiplyY(allianceColor.getPolarity()), 2000, allianceColor);
        trip2.setName("trip2");
        trip2.setDependentActions(trip1);
        autoDepot.addAction(trip2);

        // ----------------- TRIP 3 ---------------------- ~5 sec

        DepotRoundTrip trip3 = new DepotRoundTrip(opModeUtilities, driveTrain, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), farLaunchPoint.multiplyY(allianceColor.getPolarity()), 2000, allianceColor);
        trip3.setName("trip3");
        trip3.setDependentActions(trip2);
        autoDepot.addAction(trip3);

        //-------------------TRIP 4 ------------------- ~ 19qp01

//        DepotRoundTrip trip4 = new DepotRoundTrip(opModeUtilities, driveTrain, shooter, stopper, intake, Shooter.RED_TARGET_FROM_FAR.multiplyY(allianceSetup.getPolarity()), farLaunchPoint.multiplyY(allianceSetup.getPolarity()), 2000, allianceSetup);
//        trip4.setName("trip4");
//        trip4.setDependentActions(trip3);
//        autoDepot.addAction(trip4);

        // ----------------- PARK ----------------------

        PurePursuitAction park = new PurePursuitAction(driveTrain);
        park.setName("park");
        park.setDependentActions(trip3);
        park.addPoint(SHOOT_FAR_X + 400, SHOOT_FAR_Y * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
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
