package com.kalipsorobotics.decode.auto.redAuto;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.SetAutoDelayAction;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.actions.autoActions.pathActions.RampCycleAction;
import com.kalipsorobotics.actions.autoActions.pathActions.RoundTripAction;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.decode.configs.TurretConfig;
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

@Autonomous(name = "RedAutoNearSolo")
public class RedAutoNear extends KOpMode {
    KActionSet redAutoNear;
    final double FIRST_SHOOT_X = 2598;
    final double FIRST_SHOOT_Y = 441.38;
    final double SHOOT_NEAR_X = 2000; //2400
    final double SHOOT_NEAR_Y = 100; //300
    final double FINAL_SHOOT_NEAR_X = 2325; //2400
    final double FINAL_SHOOT_NEAR_Y = 75; //300
    final double THIRD_SHOOT_NEAR_X = 1900; //2400
    final double THIRD_SHOOT_NEAR_Y = 0; //300
    Point firstShotTargetPoint = new Point(Shooter.TARGET_POINT.getX() - 141.4213562373, Shooter.TARGET_POINT.getY() - 141.4213562373);

    //No polarity here because multiplied externally
    Point nearLaunchPoint =  new Point(SHOOT_NEAR_X, SHOOT_NEAR_Y);
    Point firstShootPoint = new Point(FIRST_SHOOT_X, FIRST_SHOOT_Y);
    Point lastTripLaunchPoint = new Point(FINAL_SHOOT_NEAR_X, FINAL_SHOOT_NEAR_Y);
    Point thirdShootPoint = new Point(THIRD_SHOOT_NEAR_X, THIRD_SHOOT_NEAR_Y);

    public DriveTrain driveTrain;
    Shooter shooter = null;
    Intake intake = null;
    Stopper stopper = null;
    Turret turret = null;
    TurretAutoAlign turretAutoAlign = null;
    RoundTripAction trip1 = null;
    RoundTripAction trip4 = null;
    RoundTripAction trip2 = null;
    RampCycleAction trip3 = null;

    long startTime;

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.RED;
        SharedData.setAllianceColor(allianceColor);
        TurretConfig.TICKS_INIT_OFFSET = (int) -Math.round((TurretConfig.TICKS_PER_ROTATION * TurretConfig.BIG_TO_SMALL_PULLEY) / 2); //offset by 180 deg

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
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, 3060, 712 * allianceColor.getPolarity(), -2.4049 * allianceColor.getPolarity()); //3015.93, 765.86, -2.4030
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);


        redAutoNear = new KActionSet();
        intake = new Intake(opModeUtilities);
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);

        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);
        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, allianceColor);
        turretAutoAlign.setToleranceDeg(5);

        turretAutoAlign.setToleranceDeg(3);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();




        SetAutoDelayAction setAutoDelayAction = new SetAutoDelayAction(opModeUtilities, gamepad1);
        setAutoDelayAction.setName("setAutoDelayAction");

        WaitAction delayBeforeStart = new WaitAction(setAutoDelayAction.getTimeMS());
        delayBeforeStart.setName("delayBeforeStart");
        redAutoNear.addAction(delayBeforeStart);


        // ----------------- FIRST SHOOT ----------------------
        RoundTripAction trip0 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, firstShotTargetPoint.multiplyY(allianceColor.getPolarity()), firstShootPoint.multiplyY(allianceColor.getPolarity()), 0, true);
        trip0.setName("trip0");
        trip0.getMoveToBall().clearPoints();
        trip0.getMoveToBall().addPoint(firstShootPoint.getX(), firstShootPoint.getY() * allianceColor.getPolarity(), -138.29 * allianceColor.getPolarity());
        trip0.setDependentActions(delayBeforeStart);
        trip0.setShouldShooterStop(false);
        trip0.getMoveToBall().setWithinRangeRadiusMM(400);
        trip0.getMoveToBall().setFinalAngleLockingThresholdDegree(50);
        redAutoNear.addAction(trip0);

        // ----------------- TRIP 1 (1st Spike + Lever)----------------------

        trip1 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), nearLaunchPoint.multiplyY(allianceColor.getPolarity()), 0);
        trip1.setName("trip1");
        trip1.getMoveToBall().clearPoints();
        trip1.getMoveToBall().addPoint(1950, 175 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getMoveToBall().addPoint(1950, 700 * allianceColor.getPolarity() , 90 * allianceColor.getPolarity()); //600 y
        // move to hit lever
        trip1.getMoveToBall().addPoint(1725, 900 * allianceColor.getPolarity() , 0);
        trip1.getMoveToBall().addPoint(1725, 1125 * allianceColor.getPolarity(), 0);
        trip1.getMoveToBall().addPoint(1725, 850 * allianceColor.getPolarity(), 45 * allianceColor.getPolarity());
        // move to shoot
        trip1.getMoveToBall().addPoint(SHOOT_NEAR_X, SHOOT_NEAR_Y * allianceColor.getPolarity(), 45 * allianceColor.getPolarity());
        trip1.getMoveToBall().setFinalAngleLockingThresholdDegree(45);
        trip1.setShouldShooterStop(false);
        trip1.getMoveToBall().setFinalSearchRadius(300);
        trip1.getMoveToBall().setWithinRangeRadiusMM(300);
        trip1.setDependentActions(trip0);
        redAutoNear.addAction(trip1);

        // ----------------- TRIP 2 (2nd Spike + Lever)----------------------

        trip2 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), nearLaunchPoint.multiplyY(allianceColor.getPolarity()), 0);
        trip2.setName("trip2");
        trip2.getMoveToBall().clearPoints();

        trip2.getMoveToBall().addPoint(1350, 225 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip2.getMoveToBall().addPoint(1350, 960 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        // move to lever
        trip2.getMoveToBall().addPoint(1700, 1040 * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
        // move to launch
        //trip2.getMoveToBall().addPoint(1500, (nearLaunchPoint.getY()) * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
        trip2.getMoveToBall().addPoint(nearLaunchPoint.getX(), nearLaunchPoint.getY() * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
        trip2.setDependentActions(trip1);
        trip2.setShouldShooterStop(false);
        trip2.getMoveToBall().setLookAheadRadius(200);
        trip2.getMoveToBall().setWithinRangeRadiusMM(350);
        trip2.getMoveToBall().setFinalAngleLockingThresholdDegree(50);
        redAutoNear.addAction(trip2);

        // ----------------- TRIP 3 (Tunnel) ----------------------

        handleTrip3();

        // ----------------- TRIP 4  (3rd Spike) ----------------------

        handleTrip4();

        // ----------------- TRIP 5 (park) ----------------------

        PurePursuitAction park = new PurePursuitAction(driveTrain);
        park.setDependentActions(trip4);
        park.addPoint(1800, 200 * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
        redAutoNear.addAction(park);

//        RampCycleAction trip5 = generateTunnelTrip("trip5", lastTripLaunchPoint);
//        trip5.setDependentActions(trip4);
//        redAutoNear.addAction(trip5);

        // turretAutoAlign.initBlocking();

        while(!setAutoDelayAction.getIsDone() && opModeInInit()) {
            setAutoDelayAction.updateCheckDone();
        }
        KLog.d("auto", "--------------NEAR AUTO STARTED-------------");
        waitForStart();
        long startTime = System.currentTimeMillis();
        int loopCount = 0;
        while (opModeIsActive()) {
            loopCount++;
            double elapsedSec = (System.currentTimeMillis() - startTime) / 1000.0;

            // Log overall progress every 500ms
            if (loopCount % 25 == 0) {  // Assuming ~50Hz loop rate
                KLog.d("AutoProgress", String.format("=== RedAutoNear - Time: %.1fs, Loop: %d, AutoDone: %b ===",
                        elapsedSec, loopCount, redAutoNear.getIsDone()));
                KLog.d("AutoProgress", String.format("Trips -> Trip1: %s, Trip2: %s, Trip3: %s",
                        trip1.getIsDone() ? "✓" : "...",
                        trip2.getIsDone() ? "✓" : "...",
                        trip4.getIsDone() ? "✓" : "...",
                        trip3.getIsDone() ? "✓" : "..."));
                KLog.d("AutoProgress", "NOTE: Trip3 is commented out in code - Park depends on Trip3!");
            }

            redAutoNear.updateCheckDone();
            turretAutoAlign.updateCheckDone();
            KLog.d("Odometry", "Position: " + SharedData.getOdometryWheelIMUPosition());
        }
        cleanupRobot();
    }

    public void handleTrip3() {
        trip3 = generateTunnelTrip("trip4", nearLaunchPoint);
        trip3.setDependentActions(trip2);
        redAutoNear.addAction(trip3);
    }

    public void handleTrip4() {
        trip4 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), new Point(SHOOT_NEAR_X, SHOOT_NEAR_Y * allianceColor.getPolarity()), 0);
        trip4.setName("trip3");
        trip4.setDependentActions(trip3);
        trip4.getMoveToBall().clearPoints();
        // move to intake
        trip4.getMoveToBall().addPoint(800, 320 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip4.getMoveToBall().addPoint(800, 1050 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip4.getMoveToBall().addPoint(800, 950 * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
        // move to launch
//        secondSpike.getMoveToBall().addPoint(1500, SHOOT_NEAR_Y * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
        trip4.getMoveToBall().addPoint(SHOOT_NEAR_X, SHOOT_NEAR_Y * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
        // park
        trip4.getMoveToBall().setFinalAngleLockingThresholdDegree(45);
        trip4.setShouldShooterStop(false);
        trip4.getMoveToBall().setFinalSearchRadius(300);
        trip4.getMoveToBall().setWithinRangeRadiusMM(300);
        redAutoNear.addAction(trip4);
    }

    public RampCycleAction generateTunnelTrip(String name, Point shootPoint) {
        RampCycleAction tunnelTrip = new RampCycleAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), shootPoint.multiplyY(allianceColor.getPolarity()), 0, 750);
        tunnelTrip.setName(name);
        tunnelTrip.getTripToShoot().getMoveToBall().clearPoints();
        tunnelTrip.getMoveToRamp().clearPoints();
        // hit lever
        tunnelTrip.getMoveToRamp().addPoint(1500, SHOOT_NEAR_Y * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
        tunnelTrip.getMoveToRamp().addPoint(1669, 1040 * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
        // move to tunnel
        tunnelTrip.getTripToShoot().getMoveToBall().addPoint(1401, 1169 * allianceColor.getPolarity(), 110 * allianceColor.getPolarity());
        tunnelTrip.getTripToShoot().getMoveToBall().addPoint(975, 1309 * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
        // move to shoot
        tunnelTrip.getTripToShoot().getMoveToBall().addPoint(shootPoint.getX(), shootPoint.multiplyY(allianceColor.getPolarity()).getY(), 150 * allianceColor.getPolarity());

        tunnelTrip.getMoveToRamp().setMaxTimeOutMS(2500);
        tunnelTrip.getMoveToRamp().setPathAngleTolerance(15);
        tunnelTrip.getMoveToRamp().setFinalSearchRadius(150);
        tunnelTrip.getMoveToRamp().setFinalAngleLockingThresholdDegree(20);

        tunnelTrip.getTripToShoot().setShouldShooterStop(false);
        tunnelTrip.getTripToShoot().getMoveToBall().setWithinRangeRadiusMM(300);
        tunnelTrip.getTripToShoot().getMoveToBall().setFinalAngleLockingThresholdDegree(45);

        return tunnelTrip;
    }
}