package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto;


import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.SetAutoDelayAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.WaitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions.RoundTripAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.cameraVision.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedAutoNearSolo extends KOpMode {


    KActionSet redAutoNear;
    final double FIRST_SHOOT_X = 2050;
    final double FIRST_SHOOT_Y = 100;
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
    RoundTripAction trip2 = null;
    RoundTripAction trip3 = null;
    RoundTripAction trip4 = null;
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
        trip0.getShooterReady().setName("shooterReady_trip0");
        trip0.getMoveToBall().clearPoints();
        trip0.getMoveToBall().addPoint(firstShootPoint.getX(), firstShootPoint.getY() * allianceColor.getPolarity(), -138.29 * allianceColor.getPolarity());
        trip0.setDependentActions(delayBeforeStart);
        trip0.setShouldShooterStop(false);
        trip0.getMoveToBall().setWithinRangeRadiusMM(150);
        trip0.getMoveToBall().setFinalAngleLockingThresholdDegree(50);
        redAutoNear.addAction(trip0);

        // ----------------- TRIP 1 (1st Spike)----------------------

        trip1 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), nearLaunchPoint.multiplyY(allianceColor.getPolarity()), 0);
        trip1.setName("trip1");
        trip1.getShooterReady().setName("shooterReady_trip1");
        trip1.getMoveToBall().clearPoints();
        trip1.getMoveToBall().addPoint(1975, 175 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getMoveToBall().addPoint(1975, 825 * allianceColor.getPolarity() , 90 * allianceColor.getPolarity()); //600 y
        // move to shoot
        trip1.getMoveToBall().addPoint(SHOOT_NEAR_X, SHOOT_NEAR_Y * allianceColor.getPolarity(), 45 * allianceColor.getPolarity());
        trip1.getMoveToBall().setFinalAngleLockingThresholdDegree(45);
        trip1.setShouldShooterStop(false);
        trip1.getMoveToBall().setFinalSearchRadius(200);
        trip1.getMoveToBall().setWithinRangeRadiusMM(200);
        trip1.setDependentActions(trip0);
        redAutoNear.addAction(trip1);

        // ----------------- TRIP 2 (2nd Spike)----------------------

        trip2 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), nearLaunchPoint.multiplyY(allianceColor.getPolarity()), 0);
        trip2.setName("trip2");
        trip2.getShooterReady().setName("shooterReady_trip2");
        trip2.getMoveToBall().clearPoints();

        trip2.getMoveToBall().addPoint(1350, 225 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip2.getMoveToBall().addPoint(1350, 1025 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        //Lever
        trip2.getMoveToBall().addPoint(1700, 1060 * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
        // move to launch
        trip2.getMoveToBall().addPoint(nearLaunchPoint.getX(), nearLaunchPoint.getY() * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
        trip2.setDependentActions(trip1);
        trip2.setShouldShooterStop(false);
        trip2.getMoveToBall().setLookAheadRadius(200);
        trip2.getMoveToBall().setWithinRangeRadiusMM(300);
        trip2.getMoveToBall().setFinalAngleLockingThresholdDegree(50);
        redAutoNear.addAction(trip2);

        // ----------------- TRIP 3 (3rd Spike) ----------------------

        trip3 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), nearLaunchPoint.multiplyY(allianceColor.getPolarity()), 0);
        trip3.setName("trip3");
        trip3.getShooterReady().setName("shooterReady_trip3");
        trip3.setDependentActions(trip2);
        trip3.getMoveToBall().clearPoints();
        // move to intake
        trip3.getMoveToBall().addPoint(800, 275 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip3.getMoveToBall().addPoint(800, 1065 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip3.getMoveToBall().addPoint(800, 950 * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
        // move to launch
        trip3.getMoveToBall().addPoint(SHOOT_NEAR_X, SHOOT_NEAR_Y * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
        trip3.setShouldShooterStop(false);
        trip3.getMoveToBall().setLookAheadRadius(200);
        trip3.getMoveToBall().setWithinRangeRadiusMM(300);
        trip3.getMoveToBall().setFinalAngleLockingThresholdDegree(50);
        redAutoNear.addAction(trip3);
        //----------------- TRIP 4  (Depot) ----------------------

        handleTrip4();

        //----------------- TRIP 5 (Depot) ----------------------

        handleTrip5();

        //----------------- TRIP 6 (park) ---------------------

//        PurePursuitAction park = new PurePursuitAction(driveTrain);
//        park.setDependentActions(trip5);
//        park.addPoint(1625, 300 * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
//        redAutoNear.addAction(park);

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

    public void handleTrip4() {
        trip4 = generateDepotTrip("trip4", nearLaunchPoint);
        trip4.getShooterReady().setName("shooterReady_trip4");
        trip4.setDependentActions(trip3);
        redAutoNear.addAction(trip4);
    }

    public void handleTrip5() {
        trip5 = generateDepotTrip("trip5", lastTripLaunchPoint);
        trip5.getShooterReady().setName("shooterReady_trip5");
        trip5.setDependentActions(trip4);
        redAutoNear.addAction(trip5);
    }

    public RoundTripAction generateDepotTrip(String name, Point shootPoint) {
        RoundTripAction depotTrip = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), nearLaunchPoint.multiplyY(allianceColor.getPolarity()), 0);
        depotTrip.setName(name);
        depotTrip.getMoveToBall().clearPoints();
        //move to depot
        depotTrip.getMoveToBall().addPoint(900, 1125 * allianceColor.getPolarity(), 160 * allianceColor.getPolarity());
        depotTrip.getMoveToBall().addPoint(300, 1125 * allianceColor.getPolarity(), 160 * allianceColor.getPolarity());
        // move to shoot
        depotTrip.getIntakeFullAction().getRunIntakeTime().setTimeMS(5000);
        depotTrip.getMoveToBall().addPoint(shootPoint.getX(), shootPoint.multiplyY(allianceColor.getPolarity()).getY(), 150 * allianceColor.getPolarity());

        depotTrip.getMoveToBall().setMaxTimeOutMS(2000);
        depotTrip.getMoveToBall().setFinalSearchRadius(150);
        depotTrip.getMoveToBall().setFinalAngleLockingThresholdDegree(20);

        depotTrip.setShouldShooterStop(false);
        depotTrip.getMoveToBall().setMaxTimeOutMS(6000);
        depotTrip.getMoveToBall().setWithinRangeRadiusMM(350);
        depotTrip.getMoveToBall().setFinalAngleLockingThresholdDegree(45);

        return depotTrip;
    }

}
