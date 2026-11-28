package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.SetAutoDelayAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.WaitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.FullFlowerAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeFullAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeModuleLower;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShootAllAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.configs.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.ShooterRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.IPurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.BallVision;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;

@Autonomous
public class ShootSecondAuto extends KOpMode {
    KActionSet shootSecondAuto;
    public DriveTrain driveTrain;
    Shooter shooter = null;
    Intake intake = null;
    Stopper stopper = null;
    Turret turret = null;
    FlowerMech flowerMech = null;
    TurretAutoAlign turretAutoAlign = null;
    BallVision ballVision = null;

    final double waitForShooterRunMS = 0.0;
    final double flowerIntakeMS = 2500;            // TODO-measure: longest the intake runs on a flower trip (it also stops once backed out)
    final double visionPickupTimeoutMS = 6000;     // TODO-measure: trip 4 gives up picking up after this, to leave time to shoot and park
    final BallVision.Detector ballDetector = BallVision.Detector.TFLITE; // or COLOR_BLOB (tune its HSV first)
    final Position ROBOT_START_POS = new Position(784, 181, Math.toRadians(0)); // faces -y (into the field); theta is RADIANS. -90 confirmed from 2026-11-26 TeleOp drive log
    final Point PARK_POINT = new Point(275, 1275);
    final double AUTO_LENGTH_MS = 30000;
    static final boolean USE_LIMELIGHT = true; // goal-state tracking only (up Cell / tipping); autos never rezero odometry
    final double FORCE_PARK_MS_LEFT = 3000;   // TODO-check: enough time to drive to PARK_POINT from anywhere?

    // Flowers sit 63.5 mm off the wall, so the robot stops on the field side (+), not past the wall (-).
    static final private double FLOWER_INTAKE_OFFSET_MM = 280 + 250;
    static final private double FLOWER_EXIT_OFFSET_MM = 400;
    static public final Position FLOWER_INTAKE_POSITION_1 = new Position(FieldConfig.FLOWER_1_POINT.getX(), FieldConfig.FLOWER_1_POINT.getY() + FLOWER_INTAKE_OFFSET_MM, Math.toRadians(90));
    final static public Position FLOWER_EXIT_POSITION_1 = new Position(FieldConfig.FLOWER_1_POINT.getX(), FieldConfig.FLOWER_1_POINT.getY() + FLOWER_EXIT_OFFSET_MM, Math.toRadians(90));
    static public final Position FLOWER_INTAKE_POSITION_2 = new Position(FieldConfig.FLOWER_2_POINT.getX() + FLOWER_INTAKE_OFFSET_MM, FieldConfig.FLOWER_2_POINT.getY(), Math.toRadians(0));
    public final static Position FLOWER_EXIT_POSITION_2 = new Position(FieldConfig.FLOWER_2_POINT.getX() + FLOWER_EXIT_OFFSET_MM, FieldConfig.FLOWER_2_POINT.getY(), Math.toRadians(0));


    @Override
    protected void initializeRobotConfig() {
        super.initializeRobotConfig();
        this.allianceColor = AllianceColor.RED;
        SharedData.setAllianceColor(allianceColor);
    }

    @Override
    protected void initializeRobot() {
        super.initializeRobot();
        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);
        OpModeUtilities.startOdometry(opModeUtilities, odoExecutorService, driveTrain, imuModule, ROBOT_START_POS);
        shootSecondAuto = new KActionSet();
        shootSecondAuto.setName("shootSecondAuto");
        intake = new Intake(opModeUtilities);
        flowerMech = intake.getFlower();
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);
        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);
        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret); // follows SharedData.getTargetPoint() (Cell on the robot's end); each ShooterRun names its own Cell
        if (USE_LIMELIGHT) startAprilTagDetection(turret);
        // No BeamBreakCountAction: the beam breaks are not on the robot yet (V2), and building it needs their sensors in the
        // hardware map. Trip 4 knows the intake is full from the intake motor current instead (VisionPickupAction).

        flowerMech.getServo().setPosition(FlowerMech.UP_POS);
        // Camera for trip 4: every ball it sees goes into SharedData.getBallInformation() (field mm).
        SharedData.resetBallInformation();
        ballVision = BallVision.start(hardwareMap, ballDetector, false);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        //============================TRIP 0==========================
        // The delay is picked below, after every path is built, so the paths can plan while the driver picks it.
        WaitAction delayBeforeStart = new WaitAction(5000);
        delayBeforeStart.setName("delayBeforeStart");
        shootSecondAuto.addAction(delayBeforeStart);


        // Preload goes into the up Cell, the one with the 3 nectar (Cell A). From the start pose the turret also picks Cell A (SharedData.getTargetPoint).
        // Each ShootAllAction picks its own Hive Cell with setTargetPoint (TARGET_POINT_1 = Cell A, TARGET_POINT_2 = Cell B)
        ShooterRun shooterRun = new ShooterRun(opModeUtilities, shooter, ShooterConfig.TARGET_POINT_2, ROBOT_START_POS.toPoint());
        shooterRun.setShooterRunMode(ShooterRunMode.STOP);
        shooterRun.setName("shooterRun");
        shooterRun.setDependentActions(delayBeforeStart);
        shootSecondAuto.addAction(shooterRun);

//        IPurePursuitAction trip0GoToShoot = new AdaptivePurePursuitAction(driveTrain);
//        trip0GoToShoot.setMaxTimeOutMS(4001); // give up if blocked, so the robot never keeps pushing (TODO-measure)
//        trip0GoToShoot.setName("trip3GoToShoot");
//        trip0GoToShoot.addPosition(FLOWER_INTAKE_POSITION_1.getX(), FLOWER_INTAKE_POSITION_1.getY() + 200, FLOWER_INTAKE_POSITION_1.getTheta());
//        trip0GoToShoot.setDependentActions();
//        shootSecondAuto.addAction(trip0GoToShoot);

        ShootAllAction trip0Shoot = new ShootAllAction(turret, stopper, intake, shooter, shooterRun, turretAutoAlign);
        trip0Shoot.setName("trip0Shoot");
        trip0Shoot.setTargetPoint(ShooterConfig.TARGET_POINT_2); // Cell A
        trip0Shoot.setDependentActions(delayBeforeStart);
        trip0Shoot.setShooterReadyTimeoutMs(3500);
        shootSecondAuto.addAction(trip0Shoot);

//        FlowerLower flowerLowerTrip1 = new FlowerLower(flowerMech, IntakeConfig.FLOWER_TIMEOUT_MS);
//        flowerLowerTrip1.setName("flowerLowerTrip1");
//        flowerLowerTrip1.setDependentActions(delayBeforeStart);
//        redThreeTipSoloAuto.addAction(flowerLowerTrip1);

        IntakeModuleLower intakeModuleLowerFront = new IntakeModuleLower(intake.getModule(IntakeSide.FRONT));
        intakeModuleLowerFront.setName("intakeModuleLowerFront");
        intakeModuleLowerFront.setDependentActions(delayBeforeStart);
        shootSecondAuto.addAction(intakeModuleLowerFront);

        IntakeModuleLower intakeModuleLowerBack = new IntakeModuleLower(intake.getModule(IntakeSide.BACK));
        intakeModuleLowerBack.setName("intakeModuleLowerBack");
        intakeModuleLowerBack.setDependentActions(delayBeforeStart);
        shootSecondAuto.addAction(intakeModuleLowerBack);
        //END OF TRIP 0


        //TRIP 1
        KServoAutoAction flowerLowerTrip1 = new KServoAutoAction(flowerMech.getServo(), FlowerMech.DOWN_POS);
        flowerLowerTrip1.setName("flowerLowerTrip1");
        flowerLowerTrip1.setDependentActions(trip0Shoot);
        shootSecondAuto.addAction(flowerLowerTrip1);

        FullFlowerAction trip1FlowerIntake = new FullFlowerAction(stopper, intake, flowerIntakeMS, IntakeConfig.intakePower, flowerMech, driveTrain, FLOWER_INTAKE_POSITION_1, FLOWER_EXIT_POSITION_1);
        trip1FlowerIntake.setName("trip1FlowerIntake");
//        trip3FlowerIntake.getFlowerLower().setIsDone(true);
        trip1FlowerIntake.setDependentActions(trip0Shoot);
        shootSecondAuto.addAction(trip1FlowerIntake);

        ShootAllAction trip1Shoot = new ShootAllAction(turret, stopper, intake, shooter, shooterRun, turretAutoAlign);
        trip1Shoot.setName("trip1Shoot");
        trip1Shoot.setTargetPoint(ShooterConfig.TARGET_POINT_2); // Cell B
        trip1Shoot.setDependentActions(trip1FlowerIntake);
        shootSecondAuto.addAction(trip1Shoot);
        //END OF TRIP 1

        //TRIP 2
        WaitAction waitBeforeRaise = new WaitAction(1200);
        waitBeforeRaise.setName("waitBeforeRaise");
        waitBeforeRaise.setDependentActions(trip1Shoot);
        shootSecondAuto.addAction(waitBeforeRaise);

        KServoAutoAction flowerRaiseTrip3 = new KServoAutoAction(flowerMech.getServo(), FlowerMech.UP_POS);
        flowerRaiseTrip3.setName("flowerRaiseTrip3");
        flowerRaiseTrip3.setDependentActions(waitBeforeRaise); // not delayBeforeStart: it would fight flowerLowerTrip1 over the servo
        shootSecondAuto.addAction(flowerRaiseTrip3);

        IPurePursuitAction trip2GoToGarden = new AdaptivePurePursuitAction(driveTrain);
        trip2GoToGarden.setMaxTimeOutMS(11000); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        trip2GoToGarden.setName("trip2GoToGarden");
        trip2GoToGarden.addPosition(1500,900,180);
        trip2GoToGarden.addPosition(1500,900,270);
        trip2GoToGarden.addPosition(1500,2650,270);
        trip2GoToGarden.addPosition(600,3426,0);
        trip2GoToGarden.addPosition(270,3426,0);

        trip2GoToGarden.setDependentActions(trip1Shoot);
        shootSecondAuto.addAction(trip2GoToGarden);

        IntakeFullAction trip2IntakeFullAction = new IntakeFullAction(stopper, intake, 1_000_000, IntakeConfig.intakePower);
        trip2IntakeFullAction.setName("trip2IntakeFullAction");
        trip2IntakeFullAction.setDependentActions(trip1Shoot);
        shootSecondAuto.addAction(trip2IntakeFullAction);

        // Sweep done: stop the intake now, so its timer does not keep it running into the shot (same as FullFlowerAction).
        IntakeStop trip2StopIntake = new IntakeStop(trip2IntakeFullAction);
        trip2StopIntake.setName("trip2StopIntake");
        trip2StopIntake.setDependentActions(trip2GoToGarden);
        shootSecondAuto.addAction(trip2StopIntake);

        ShootAllAction trip2GardenShoot = new ShootAllAction(turret, stopper, intake, shooter, shooterRun, turretAutoAlign);
        trip2GardenShoot.setName("trip2GardenShoot");
        trip2GardenShoot.setTargetPoint(ShooterConfig.TARGET_POINT_1); // Cell A
        trip2GardenShoot.setDependentActions(trip2GoToGarden);
        shootSecondAuto.addAction(trip2GardenShoot);
        //END OF TRIP 2

        //TRIP 3
        FullFlowerAction trip3FlowerIntake = new FullFlowerAction(stopper, intake, flowerIntakeMS, IntakeConfig.intakePower, flowerMech, driveTrain, FLOWER_INTAKE_POSITION_2, FLOWER_EXIT_POSITION_2);
        trip3FlowerIntake.setName("trip3FlowerIntake");
        trip3FlowerIntake.setDependentActions(trip2GardenShoot);
        shootSecondAuto.addAction(trip3FlowerIntake);

        IPurePursuitAction trip3GoToShoot = new AdaptivePurePursuitAction(driveTrain);
        trip3GoToShoot.setMaxTimeOutMS(4001); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        trip3GoToShoot.setName("trip3GoToShoot");
        trip3GoToShoot.addPosition(850, 3426, 0);
        trip3GoToShoot.setDependentActions(trip3FlowerIntake);
        shootSecondAuto.addAction(trip3GoToShoot);

        ShootAllAction trip3Shoot = new ShootAllAction(turret, stopper, intake, shooter, shooterRun, turretAutoAlign);
        trip3Shoot.setName("trip3Shoot");
        trip3Shoot.setTargetPoint(ShooterConfig.TARGET_POINT_1); // Cell A
        trip3Shoot.setDependentActions(trip3GoToShoot);
        shootSecondAuto.addAction(trip3Shoot);
        //END OF TRIP 3


        IPurePursuitAction trip4Park = new AdaptivePurePursuitAction(driveTrain);
        trip4Park.setMaxTimeOutMS(3000); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        trip4Park.setName("trip4Park");
        trip4Park.addPosition(PARK_POINT.getX(), PARK_POINT.getY(), 180); // TODO-check: park heading
        trip4Park.setDependentActions(trip3Shoot);
        shootSecondAuto.addAction(trip4Park);
        //END OF TRIP 4

        // FORCE PARK: with FORCE_PARK_MS_LEFT left in the match, drop everything and park
        IPurePursuitAction forcePark = new AdaptivePurePursuitAction(driveTrain);
        forcePark.setMaxTimeOutMS(3000); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        forcePark.setName("forcePark");
        forcePark.addPosition(PARK_POINT.getX(), PARK_POINT.getY(), 180); // TODO-check: park heading

        // Pick the start delay. Plan the paths meanwhile: if START is pressed here, a path left unplanned
        // gets planned during the match, and the robot sits still until it is done.
        SetAutoDelayAction setAutoDelayAction = new SetAutoDelayAction(opModeUtilities, gamepad1, 5000);
        setAutoDelayAction.setName("setAutoDelayAction");
        while (!setAutoDelayAction.getIsDone() && opModeInInit()) {
            AdaptivePurePursuitAction.runPrecomputeStepForAll();
            // Added before setAutoDelayAction, which sends the telemetry.
            telemetry.addLine(AdaptivePurePursuitAction.precomputeStatus());
            addGoalStateTelemetry(); // test the goal-state tracking before the match starts
            setAutoDelayAction.updateCheckDone();
            initLoop();
        }
        delayBeforeStart.setWaitTimeMS(setAutoDelayAction.getTimeMS());

        waitForStartPrecomputingPaths();
        ElapsedTime matchTimer = new ElapsedTime();   // starts when the match starts
        boolean forceParking = false;

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            intake.update();
            turretAutoAlign.updateCheckDone(); // the turret only moves while its action is updated

            boolean timeToPark = matchTimer.milliseconds() >= AUTO_LENGTH_MS - FORCE_PARK_MS_LEFT;
            if (timeToPark && !forceParking) {
                forceParking = true;
                stopEverythingButDrive();   // the auto actions are not updated any more, so nothing turns them back on
                KLog.d("shootSecondAuto", "force park at " + matchTimer.milliseconds() + " ms");
            }

            if (forceParking) {
                forcePark.updateCheckDone();   // the auto actions are not updated any more
            } else {
                shootSecondAuto.updateCheckDone();
            }
            KLog.d("Odometry", () -> "Position: " + SharedData.getOdometryWheelIMUPosition());
            addGoalStateTelemetry(); // up Cell changes also log as Vision_Limelight event=goalState
            telemetry.update();
        }
        shooter.stop();
        ballVision.close();
        cleanupRobot();
    }

    /** Aim the turret at the target from the start pose during init, so trip 0 shoots without waiting on the turret. */
    @Override
    protected void initLoop() {
        turretAutoAlign.updateCheckDone();
    }

    /** Shooter, intake and turret off: parking only needs the drivetrain. */
    private void stopEverythingButDrive() {
        shooter.stop();
        intake.stop();
        turretAutoAlign.stop();
        KLog.d("shootSecondAuto", "stopped shooter, intake and turret before parking");
    }
}

