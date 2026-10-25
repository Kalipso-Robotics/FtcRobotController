package org.firstinspires.ftc.teamcode.kalipsorobotics.simdemo;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions.decode.RoundTripAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeFullAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;

/**
 * A SIM-ONLY auto that shows the ball handling working end to end. It is not robot code and it does not replace RedTippingAuto
 * (whose shoot and intake lines are all commented out). It is built from the same classes the robot uses: the DECODE-era
 * RoundTripAction (drive and intake along a path, then ShooterRun, ShooterReady, TurretReadyAuto and PushBall), Shooter, Intake,
 * Stopper, Turret and TurretAutoAlign. When the team connects those lines in its own auto, that auto behaves the same way here.
 *
 * What it does, in the Red frame:
 *   1. shoot the 4 preloaded pollen into the red Hive's up Cell (with its 3 nectar that tips the Hive and the balls spill out)
 *   2. drive into the red Garden (4 pollen in a line), then to launch point B and shoot them into the Cell that is up now
 *   3. pick up the 4 pollen in Flower B, shoot them into the same Cell (8 pollen tips it)
 * The turret aims at the target point of each trip (the DECODE code aims at TurretConfig.X/Y_INIT_SETUP_MM, so those are set per trip).
 */
public class BallShowcaseAuto extends KOpMode {

    /**
     * The start pose. RedTippingAuto starts 171.5 mm from the wall, but with RobotFootprint's placeholder body (229 mm behind the
     * centre) that puts the robot 58 mm inside the wall, so odometry is 58 mm off for the whole run and the robot cannot reach a
     * point next to an obstacle. The showcase starts touching the wall with the body fully inside the field instead.
     * sim/opmodes.json must give the same pose.
     */
    static final double START_X_MM = 1389.5;
    static final double START_Y_MM = 3657.6 - 229 - 3;

    private KActionSet showcase;
    private DriveTrain driveTrain;
    private Shooter shooter;
    private Intake intake;
    private Stopper stopper;
    private Turret turret;
    private TurretAutoAlign turretAutoAlign;

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.RED;
        SharedData.setAllianceColor(allianceColor);
        TurretConfig.TICKS_INIT_OFFSET = 0; // the turret starts facing straight ahead
        TurretConfig.X_INIT_SETUP_MM = FieldConfig.aTargetPoint.getX(); // the turret aims at the first Cell
        TurretConfig.Y_INIT_SETUP_MM = FieldConfig.aTargetPoint.getY();
    }

    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);

        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, START_X_MM, START_Y_MM, Math.toRadians(-90));
        OpModeUtilities.runOdometryExecutorService(odoExecutorService, odometry);

        showcase = new KActionSet();
        intake = new Intake(opModeUtilities);
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);
        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);
        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, allianceColor);
        turretAutoAlign.setToleranceDeg(5);
    }

    /** One trip: drive the points (intake runs along the way when asked), then shoot at the target from the last point. */
    private RoundTripAction trip(String name, Point target, Point launch, boolean runIntake, double[][] points) {
        RoundTripAction trip = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake,
                target, launch, 0, runIntake, false, true);
        trip.setName(name);
        trip.getMoveToBall().clearPoints();
        for (double[] p : points) {
            trip.getMoveToBall().addPoint(p[0], p[1], p[2]);
        }
        trip.setShouldShooterStop(false);
        trip.getPurePursuitReadyShooting().setDistanceThresholdMM(250);
        return trip;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        Point launchA = FieldConfig.aLaunchPoint, targetA = FieldConfig.aTargetPoint;
        Point launchB = FieldConfig.bLaunchPoint, targetB = FieldConfig.bTargetPoint;

        // 1. the preload into Cell A
        RoundTripAction preload = trip("preload", targetA, launchA, false, new double[][]{{launchA.getX(), launchA.getY(), -90}});
        showcase.addAction(preload);

        // 2. the red Garden: its 4 pollen sit in a line along the bottom wall, so drive straight in (front toward the wall) and the
        //    front of the robot meets all four at once. Then to launch point B and into the Cell that is up after the tip.
        //    (The robot code stops the intake 900 mm before the end of a path, so the way back has to be long enough.)
        RoundTripAction garden = trip("garden", targetB, launchB, true, new double[][]{
                {700, 2900, 90}, {235, 3100, 90}, {235, 3320, 90},
                {600, 3000, -90}, {800, 2000, -90}, {900, 1100, -90}, {launchB.getX(), launchB.getY(), 90}});
        garden.setDependentActions(preload);
        showcase.addAction(garden);

        // 3. Flower B: the intake runs while the robot drives in from the field side, then it goes back out and shoots.
        //    RoundTripAction stops its intake 900 mm before the end of the path, and this trip starts and ends at launch point B,
        //    so the intake is its own real action here, running for the time it takes to reach the Flower.
        IntakeFullAction flowerIntake = new IntakeFullAction(stopper, intake, 2600, IntakeConfig.intakePower);
        flowerIntake.setName("flowerIntake");
        flowerIntake.setDependentActions(garden);
        showcase.addAction(flowerIntake);
        RoundTripAction flower = trip("flowerB", targetB, launchB, false, new double[][]{
                {1219, 1000, -90}, {1219, 700, -90}, {1219, 540, -90},
                {900, 1000, -90}, {1300, 1050, 90}, {launchB.getX(), launchB.getY(), 90}});
        flower.setDependentActions(garden);
        showcase.addAction(flower);

        waitForStartPrecomputingPaths();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            showcase.updateCheckDone();
            KLog.d("BallShowcase", () -> "position " + SharedData.getOdometryWheelIMUPosition());
        }
        cleanupRobot();
    }
}
