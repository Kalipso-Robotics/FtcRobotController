package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.SetAutoDelayAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.WaitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions.RampCycleAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions.RoundTripAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.cameraVision.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class RedAutoNear extends KOpMode {
    KActionSet redAutoNear;
    final double FIRST_SHOOT_X = 2598 - 300;
    final double FIRST_SHOOT_Y = 441.38 - 300;
    final double SHOOT_NEAR_X = 2050; //2400
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
        OpModeUtilities.runOdometryExecutorService(odoExecutorService, odometry);


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
        trip0.getMoveToBall().setWithinRangeRadiusMM(200);
        trip0.getMoveToBall().setFinalAngleLockingThresholdDegree(50);
        redAutoNear.addAction(trip0);

        // ----------------- TRIP 1 (2nd spike)----------------------

        trip1 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), nearLaunchPoint.multiplyY(allianceColor.getPolarity()), 0);
        trip1.setName("trip1");
        trip1.getMoveToBall().clearPoints();

        trip1.getMoveToBall().addPoint(1350, 225 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getMoveToBall().addPoint(1350, 960 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());

        //lever
        trip1.getMoveToBall().addPoint(1575, 1085 * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
        trip1.getMoveToBall().addPoint(1450, 275 * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());

        // move to launch
//        trip1.getMoveToBall().addPoint(1350, 400 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getMoveToBall().addPoint(nearLaunchPoint.getX(), nearLaunchPoint.getY() * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());

        trip1.getMoveToBall().setFinalAngleLockingThresholdDegree(45);
        trip1.setShouldShooterStop(false);
        trip1.getMoveToBall().setFinalSearchRadius(200);
        trip1.getMoveToBall().setWithinRangeRadiusMM(200);
        trip1.setDependentActions(trip0);
        redAutoNear.addAction(trip1);

        // ----------------- TRIP 2 (3rd Spike)----------------------

        trip2 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), nearLaunchPoint.multiplyY(allianceColor.getPolarity()), 0);
        trip2.setName("trip2");
        trip2.getMoveToBall().clearPoints();

        trip2.getMoveToBall().addPoint(775, 250 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip2.getMoveToBall().addPoint(775, 1085 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
//        trip2.getMoveToBall().addPoint(500, 1140 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
//        trip2.getMoveToBall().addPoint(500, 800 * allianceColor.getPolarity(), 135 * allianceColor.getPolarity());

        trip2.getMoveToBall().addPoint(nearLaunchPoint.getX(), nearLaunchPoint.getY() * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());

        trip2.setDependentActions(trip1);
        trip2.setShouldShooterStop(false);
        trip2.getMoveToBall().setLookAheadRadius(200);
        trip2.getMoveToBall().setWithinRangeRadiusMM(200);
        trip2.getMoveToBall().setFinalAngleLockingThresholdDegree(50);
        redAutoNear.addAction(trip2);

        // ----------------- TRIP 3 (Tunnel) ----------------------

        handleTrip3();

        // ----------------- TRIP 4  (1st Spike) ----------------------

        handleTrip4();

        // ----------------- TRIP 5 (park) ----------------------

        PurePursuitAction park = new PurePursuitAction(driveTrain);
        park.setDependentActions(trip4);
        park.addPoint(1725, 250 * allianceColor.getPolarity(), 45 * allianceColor.getPolarity());
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
        trip4 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), nearLaunchPoint.multiplyY(allianceColor.getPolarity()), 0);
        trip4.getMoveToBall().clearPoints();

        trip4.getMoveToBall().addPoint(1950, 175 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip4.getMoveToBall().addPoint(1950, 850 * allianceColor.getPolarity() , 90 * allianceColor.getPolarity()); //600 y

        // move to shoot
        trip4.getMoveToBall().addPoint(nearLaunchPoint.getX(), nearLaunchPoint.getY() * allianceColor.getPolarity(), 45 * allianceColor.getPolarity());

        trip4.setDependentActions(trip3);
        trip4.setShouldShooterStop(false);
        trip4.getMoveToBall().setLookAheadRadius(125);
        trip4.getMoveToBall().setWithinRangeRadiusMM(200);
        trip4.getMoveToBall().setFinalAngleLockingThresholdDegree(50);
        redAutoNear.addAction(trip4);
    }

    public RampCycleAction generateTunnelTrip(String name, Point shootPoint) {
        RampCycleAction tunnelTrip = new RampCycleAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), shootPoint.multiplyY(allianceColor.getPolarity()), 0, 800);
        tunnelTrip.setName(name);
        tunnelTrip.getTripToShoot().getMoveToBall().clearPoints();
        tunnelTrip.getMoveToRamp().clearPoints();
        tunnelTrip.getMoveToRamp().addPoint(1550, 275 * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());

        // hit lever
        tunnelTrip.getMoveToRamp().addPoint(1550, 1070 * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
        // move to tunnel
//        tunnelTrip.getTripToShoot().getMoveToBall().addPoint(1450, 1169 * allianceColor.getPolarity(), 140 * allianceColor.getPolarity());
        tunnelTrip.getTripToShoot().getMoveToBall().addPoint(1400, 1025 * allianceColor.getPolarity(), 120 * allianceColor.getPolarity());
        tunnelTrip.getTripToShoot().getMoveToBall().addPoint(1300, 1085 * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
        tunnelTrip.getTripToShoot().getMoveToBall().addPoint(975, 1225 * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
        tunnelTrip.getTripToShoot().getMoveToBall().addPoint(975, 300 * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());

        // move to shoot
        tunnelTrip.getTripToShoot().getMoveToBall().addPoint(shootPoint.getX(), shootPoint.multiplyY(allianceColor.getPolarity()).getY(), 150 * allianceColor.getPolarity());

        tunnelTrip.getMoveToRamp().setMaxTimeOutMS(2000);
        tunnelTrip.getMoveToRamp().setFinalSearchRadius(150);
        tunnelTrip.getMoveToRamp().setFinalAngleLockingThresholdDegree(20);

        tunnelTrip.getTripToShoot().setShouldShooterStop(false);
        tunnelTrip.getTripToShoot().getMoveToBall().setMaxTimeOutMS(6000);
        tunnelTrip.getTripToShoot().getMoveToBall().setWithinRangeRadiusMM(200);
        tunnelTrip.getTripToShoot().getMoveToBall().setFinalAngleLockingThresholdDegree(45);

        return tunnelTrip;
    }
}