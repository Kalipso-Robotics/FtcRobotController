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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedAutoNearRampThirdSpike extends KOpMode {
    KActionSet redAutoNear;
    final double FIRST_SHOOT_X = 2400;
    final double FIRST_SHOOT_Y = 128;
    final double SHOOT_NEAR_X = 1765; //2400
    final double SHOOT_NEAR_Y = 0; //300
    final double FINAL_SHOOT_NEAR_X = 2475;
    final double FINAL_SHOOT_NEAR_Y = 0; //300
    final double THIRD_SHOOT_NEAR_X = 1900; //2400
    final double THIRD_SHOOT_NEAR_Y = 0; //300
    Point firstShotTargetPoint = Shooter.TARGET_POINT;
    Point nearLaunchPoint =  new Point(SHOOT_NEAR_X, SHOOT_NEAR_Y);
    Point firstShootPoint = new Point(FIRST_SHOOT_X, FIRST_SHOOT_Y);
    Point lastTripLaunchPoint = new Point(FINAL_SHOOT_NEAR_X, FINAL_SHOOT_NEAR_Y);

    public DriveTrain driveTrain;
    Shooter shooter = null;
    Intake intake = null;
    Stopper stopper = null;
    Turret turret = null;
    TurretAutoAlign turretAutoAlign = null;
    RoundTripAction trip0 = null;
    RoundTripAction trip1 = null;
    RampCycleAction trip2 = null;
    RampCycleAction trip3 = null;
    RoundTripAction trip4Spike = null;
    RampCycleAction trip4Ramp = null;

    RoundTripAction trip5 = null;

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

        turretAutoAlign.setToleranceDeg(2.5);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        //No polarity here because multiplied externally



        SetAutoDelayAction setAutoDelayAction = new SetAutoDelayAction(opModeUtilities, gamepad1);
        setAutoDelayAction.setName("setAutoDelayAction");

        WaitAction delayBeforeStart = new WaitAction(setAutoDelayAction.getTimeMS());
        delayBeforeStart.setName("delayBeforeStart");
        redAutoNear.addAction(delayBeforeStart);


        // ----------------- FIRST SHOOT ----------------------
        trip0 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, firstShotTargetPoint.multiplyY(allianceColor.getPolarity()), firstShootPoint.multiplyY(allianceColor.getPolarity()), 0, true);
        trip0.setName("trip0");
        trip0.getShooterReady().setName("shooterReady_trip0");
        trip0.getMoveToBall().clearPoints();
        trip0.getMoveToBall().addPoint(firstShootPoint.getX(), firstShootPoint.getY() * allianceColor.getPolarity(), -138.29 * allianceColor.getPolarity());
        trip0.setDependentActions(delayBeforeStart);
        trip0.setShouldShooterStop(false);
        trip0.getMoveToBall().setWithinRangeRadiusMM(400);
        trip0.getMoveToBall().setFinalAngleLockingThresholdDegree(50);
        trip0.getMoveToBall().setFinalSearchRadius(150);
        redAutoNear.addAction(trip0);

        // ----------------- TRIP 1 ----------------------

        handleTrip1();

        // ----------------- TRIP 2 ---------------------- ramp

        trip2 = generateRampTrip();
        trip2.setName("trip2");
        trip2.setDependentActions(trip1);
        redAutoNear.addAction(trip2);

        // ----------------- TRIP 3 ----------------------

        trip3 = generateRampTrip();
        trip3.setName("trip3");
        trip3.setDependentActions(trip2);
        redAutoNear.addAction(trip3);

        // ----------------- TRIP 4 ----------------------

        handleTrip4();

        // ----------------- TRIP 5 ----------------------

        handleTrip5();

        // ----------------- TRIP 6 ----------------------

//        PurePursuitAction park = new PurePursuitAction(driveTrain);
//        park.setDependentActions(trip5);
//        park.addPoint(1800, 200 * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
//        redAutoNear.addAction(park);



        // turretAutoAlign.initBlocking();

        while(!setAutoDelayAction.getIsDone() && opModeInInit()) {
            setAutoDelayAction.updateCheckDone();
        }
        KLog.d("auto", "--------------NEAR AUTO STARTED-------------");
        waitForStart();
        while (opModeIsActive()) {
            redAutoNear.updateCheckDone();
            turretAutoAlign.updateCheckDone();
            KLog.d("Odometry", "Position: " + SharedData.getOdometryWheelIMUPosition());
        }
        cleanupRobot();
    }

    protected void handleTrip1() {
        trip1 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), nearLaunchPoint.multiplyY(allianceColor.getPolarity()), 0);
        trip1.setName("trip1");
        trip1.getShooterReady().setName("shooterReady_trip1");
        trip1.getMoveToBall().clearPoints();
        trip1.getMoveToBall().addPoint(1375, 225 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getMoveToBall().addPoint(1375, 1025 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        // move to launch
        trip1.getMoveToBall().addPoint(1500, nearLaunchPoint.getY() * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
        trip1.getMoveToBall().addPoint(nearLaunchPoint.getX(), nearLaunchPoint.getY() * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
        trip1.setDependentActions(trip0);
        trip1.setShouldShooterStop(false);
        trip1.getMoveToBall().setWithinRangeRadiusMM(300);
        trip1.getMoveToBall().setMaxTimeOutMS(9000);
        trip1.getMoveToBall().setFinalAngleLockingThresholdDegree(50);
        trip1.getMoveToBall().setPathAngleTolerance(50);
        trip1.getMoveToBall().setFinalSearchRadius(200);
        redAutoNear.addAction(trip1);
    }

    public void handleTrip3() { //new ramp 2
        trip3 = generateRampTrip();
        trip3.setName("trip3");
        trip3.setDependentActions(trip2);
        redAutoNear.addAction(trip3);
    }


    protected void handleTrip4() {
        trip4Ramp = null;
        trip4Spike = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), new Point(FINAL_SHOOT_NEAR_X, FINAL_SHOOT_NEAR_Y * allianceColor.getPolarity()), 0, true);
        trip4Spike.setName("trip4Spike");
        trip4Spike.getShooterReady().setName("shooterReady_trip4Spike");
        // move to intake
        trip4Spike.setDependentActions(trip3);

        trip4Spike.getMoveToBall().clearPoints();
        trip4Spike.getMoveToBall().clearPoints();
        trip4Spike.getMoveToBall().addPoint(775, 370 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip4Spike.getMoveToBall().addPoint(775, 1000 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip4Spike.getMoveToBall().addPoint(775, 850 * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
        trip4Spike.getMoveToBall().addPoint(SHOOT_NEAR_X, SHOOT_NEAR_Y * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
        trip4Spike.getMoveToBall().setEnablePowerScalingForPath(true);
        trip4Spike.setShouldShooterStop(false);
        trip4Spike.getMoveToBall().setWithinRangeRadiusMM(300);
        trip4Spike.getMoveToBall().setFinalSearchRadius(200);
        trip4Spike.getMoveToBall().setFinalAngleLockingThresholdDegree(45);
        redAutoNear.addAction(trip4Spike);
    }


    public void handleTrip5() {
        trip5 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), new Point(THIRD_SHOOT_NEAR_X, THIRD_SHOOT_NEAR_Y * allianceColor.getPolarity()), 0);
        trip5.setName("trip4");
        // move to intake
        if (trip4Spike != null) {
            trip5.setDependentActions(trip4Spike);
        } else {
            trip5.setDependentActions(trip4Ramp);
        }
        trip5.getMoveToBall().clearPoints();
        // move to first spike
        trip5.getMoveToBall().addPoint(1950, 175 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        // eat balls
        trip5.getMoveToBall().addPoint(1950, 745 * allianceColor.getPolarity() , 90 * allianceColor.getPolarity()); //600 y

        trip5.getMoveToBall().setFinalAngleLockingThresholdDegree(45);
        trip5.setShouldShooterStop(false);
        trip5.getMoveToBall().setFinalSearchRadius(300);
        trip5.getMoveToBall().setWithinRangeRadiusMM(300);
        // move to launch
        trip5.getMoveToBall().addPoint(FINAL_SHOOT_NEAR_X, FINAL_SHOOT_NEAR_Y * allianceColor.getPolarity(), 150 * allianceColor.getPolarity(), PurePursuitAction.P_XY, PurePursuitAction.P_ANGLE * 2);
        redAutoNear.addAction(trip5);
    }

    public RampCycleAction generateRampTrip() {
        RampCycleAction rampTrip = new RampCycleAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), nearLaunchPoint.multiplyY(allianceColor.getPolarity()), 0, 1000, 130);
        rampTrip.getTripToShoot().getShooterReady().setName("rampTrip");
        rampTrip.getMoveToRamp().clearPoints();
        //move to lever
//        rampTrip.getMoveToRamp().addPoint(1399.79, 925.11 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        //eat at lever
        rampTrip.getMoveToEat().addPoint(1425, 975.11 * allianceColor.getPolarity(), 67.3 * allianceColor.getPolarity()); // eating point
        rampTrip.getMoveToEat().setPathAngleTolerance(5);
        rampTrip.getMoveToEat().setLookAheadRadius(75);
        rampTrip.getMoveToEat().setFinalAngleLockingThresholdDegree(10);
        rampTrip.getMoveToEat().setFinalSearchRadius(150);
        rampTrip.getMoveToEat().setMaxTimeOutMS(3000);
        // move to launch
        rampTrip.getTripToShoot().getMoveToBall().addPoint(SHOOT_NEAR_X, SHOOT_NEAR_Y * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
        rampTrip.getTripToShoot().setShouldShooterStop(false);
        rampTrip.getTripToShoot().getMoveToBall().setMaxTimeOutMS(4000);
        rampTrip.getTripToShoot().getMoveToBall().setFinalSearchRadius(200);
        rampTrip.getTripToShoot().getMoveToBall().setWithinRangeRadiusMM(200);
        rampTrip.getTripToShoot().getMoveToBall().setFinalAngleLockingThresholdDegree(30);
        return rampTrip;
    }//
}