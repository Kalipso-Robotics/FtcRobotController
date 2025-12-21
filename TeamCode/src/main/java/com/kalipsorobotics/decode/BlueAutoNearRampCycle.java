package com.kalipsorobotics.decode;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.SetAutoDelayAction;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.actions.autoActions.pathActions.RampCycleAction;
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
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KOpMode;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RedAutoNearRampCycle")
public class BlueAutoNearRampCycle extends KOpMode {
    KActionSet blueAutoNear;
    final double FIRST_SHOOT_X = 2598;
    final double FIRST_SHOOT_Y = 441.38;
    final double SHOOT_NEAR_X = 2130; //2400
    final double SHOOT_NEAR_Y = 135; //300

    public DriveTrain driveTrain;
    Shooter shooter = null;
    Intake intake = null;
    Stopper stopper = null;
    Turret turret = null;
    TurretAutoAlign turretAutoAlign = null;

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.BLUE;
        SharedData.setAllianceColor(allianceColor);
        TurretConfig.TICKS_INIT_OFFSET = (Turret.TICKS_PER_ROTATION * Turret.BIG_TO_SMALL_PULLEY) / 2; //offset by 180 deg
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


        blueAutoNear = new KActionSet();
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
        Point nearLaunchPoint =  new Point(SHOOT_NEAR_X, SHOOT_NEAR_Y);
        Point firstShootPoint = new Point(FIRST_SHOOT_X, FIRST_SHOOT_Y);

        SetAutoDelayAction setAutoDelayAction = new SetAutoDelayAction(opModeUtilities, gamepad1);
        setAutoDelayAction.setName("setAutoDelayAction");

        WaitAction delayBeforeStart = new WaitAction(setAutoDelayAction.getTimeMS());
        delayBeforeStart.setName("delayBeforeStart");
        blueAutoNear.addAction(delayBeforeStart);


        // ----------------- FIRST SHOOT ----------------------
        RoundTripAction trip0 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), firstShootPoint.multiplyY(allianceColor.getPolarity()), 0, false);
        trip0.setName("trip0");
        trip0.getMoveToBall().addPoint(firstShootPoint.getX(), firstShootPoint.getY() * allianceColor.getPolarity(), -138.29 * allianceColor.getPolarity());
        trip0.setDependentActions(delayBeforeStart);
        blueAutoNear.addAction(trip0);

        // ----------------- TRIP 1 ----------------------

        RoundTripAction trip1 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), nearLaunchPoint.multiplyY(allianceColor.getPolarity()), 0);
        trip1.setName("trip1");
        trip1.getMoveToBall().addPoint(1970, 175 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getMoveToBall().addPoint(1970, 840 * allianceColor.getPolarity() , 90 * allianceColor.getPolarity());
        // move to hit lever
//        trip1.getMoveToBall().addPoint(1755, 1200 * allianceColor.getPolarity(), 0 * allianceColor.getPolarity());
        // move to launch
        trip1.getMoveToBall().addPoint(nearLaunchPoint.getX(), nearLaunchPoint.getY() * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
        trip1.setDependentActions(trip0);
        blueAutoNear.addAction(trip1);

        // ----------------- TRIP 2 ----------------------
        RoundTripAction trip2 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), nearLaunchPoint.multiplyY(allianceColor.getPolarity()), 0);
        trip2.setName("trip2");
        trip2.getMoveToBall().addPoint(1264.64, 509.85 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip2.getMoveToBall().addPoint(1399.00, 848.38 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip2.getMoveToBall().addPoint(1753.27, 1069.35 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip2.getMoveToBall().addPoint(nearLaunchPoint.getX(), nearLaunchPoint.getY() * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
        trip2.setDependentActions(trip1);

        blueAutoNear.addAction(trip2);

        // ----------------- TRIP 3 ----------------------
        RampCycleAction trip3 = new RampCycleAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, allianceColor);
        trip3.setDependentActions(trip2);
        blueAutoNear.addAction(trip3);

        // ----------------- TRIP 4 ----------------------
        RampCycleAction trip4 = new RampCycleAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, allianceColor);
        trip4.setDependentActions(trip3);
        blueAutoNear.addAction(trip4);

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
                        elapsedSec, loopCount, blueAutoNear.getIsDone()));
                KLog.d("AutoProgress", String.format("Trips -> Trip1: %s, Trip2: %s, Trip3: %s",
                        trip1.getIsDone() ? "✓" : "...",
                        trip2.getIsDone() ? "✓" : "...",
                        trip3.getIsDone() ? "✓" : "..."));
                //park.getIsDone() ? "✓" : "..."));
                KLog.d("AutoProgress", "NOTE: Trip3 is commented out in code - Park depends on Trip3!");
            }

            blueAutoNear.updateCheckDone();
            turretAutoAlign.updateCheckDone();
            KLog.d("Odometry", "Position: " + SharedData.getOdometryPosition());
        }
        cleanupRobot();
    }
}
