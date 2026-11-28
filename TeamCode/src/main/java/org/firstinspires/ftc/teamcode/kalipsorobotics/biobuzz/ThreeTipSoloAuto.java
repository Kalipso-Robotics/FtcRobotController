package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.SetAutoDelayAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.WaitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath.VisionPickupAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.MoveStraightPIDAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.FullFlowerAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeFullAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeModuleLower;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShootAllAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.configs.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
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
public class ThreeTipSoloAuto extends KOpMode {
    KActionSet redThreeTipSoloAuto;
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
    final Position ROBOT_START_POS = new Position(1006, 3426, Math.toRadians(0)); // faces -y (into the field); theta is RADIANS. -90 confirmed from 2026-11-26 TeleOp drive log
    final Point PARK_POINT = new Point(275, 1275);
    final double AUTO_LENGTH_MS = 30000;
    static final boolean USE_LIMELIGHT = false; // nothing in this auto reads the Limelight yet; true = start it anyway
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
        redThreeTipSoloAuto = new KActionSet();
        redThreeTipSoloAuto.setName("redThreeTipSoloAuto");
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
        WaitAction delayBeforeStart = new WaitAction(0);
        delayBeforeStart.setName("delayBeforeStart");
        redThreeTipSoloAuto.addAction(delayBeforeStart);


        // Preload goes into the up Cell, the one with the 3 nectar (Cell A). From the start pose the turret also picks Cell A (SharedData.getTargetPoint).
        // Each ShootAllAction picks its own Hive Cell with setTargetPoint (TARGET_POINT_1 = Cell A, TARGET_POINT_2 = Cell B)
        ShooterRun shooterRun = new ShooterRun(opModeUtilities, shooter, ShooterConfig.TARGET_POINT_1, ROBOT_START_POS.toPoint());
        shooterRun.setShooterRunMode(ShooterRunMode.STOP);
        shooterRun.setName("shooterRun");
        shooterRun.setDependentActions(delayBeforeStart);
        redThreeTipSoloAuto.addAction(shooterRun);

        ShootAllAction trip0Shoot = new ShootAllAction(turret, stopper, intake, shooter, shooterRun, turretAutoAlign);
        trip0Shoot.setName("trip0Shoot");
        trip0Shoot.setTargetPoint(ShooterConfig.TARGET_POINT_1); // Cell A
        trip0Shoot.setDependentActions(delayBeforeStart);
        redThreeTipSoloAuto.addAction(trip0Shoot);

//        FlowerLower flowerLowerTrip1 = new FlowerLower(flowerMech, IntakeConfig.FLOWER_TIMEOUT_MS);
//        flowerLowerTrip1.setName("flowerLowerTrip1");
//        flowerLowerTrip1.setDependentActions(delayBeforeStart);
//        redThreeTipSoloAuto.addAction(flowerLowerTrip1);

        IntakeModuleLower intakeModuleLowerFront = new IntakeModuleLower(intake.getModule(IntakeSide.FRONT));
        intakeModuleLowerFront.setName("intakeModuleLowerFront");
        intakeModuleLowerFront.setDependentActions(delayBeforeStart);
        redThreeTipSoloAuto.addAction(intakeModuleLowerFront);

        IntakeModuleLower intakeModuleLowerBack = new IntakeModuleLower(intake.getModule(IntakeSide.BACK));
        intakeModuleLowerBack.setName("intakeModuleLowerBack");
        intakeModuleLowerBack.setDependentActions(delayBeforeStart);
        redThreeTipSoloAuto.addAction(intakeModuleLowerBack);
        //END OF TRIP 0


        //TRIP 1
        IPurePursuitAction trip1GoToGarden = new AdaptivePurePursuitAction(driveTrain);
        trip1GoToGarden.setMaxTimeOutMS(4500); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        trip1GoToGarden.setName("trip3GoToGarden");
//        trip1GoToGarden.addPosition(800, 543, 102.6);
//        trip1GoToGarden.addPosition(300, 2949, 102.6);
        trip1GoToGarden.addPosition(270,3426,0);
        trip1GoToGarden.setDependentActions(trip0Shoot);
        redThreeTipSoloAuto.addAction(trip1GoToGarden);
//
//        MoveStraightPIDAction trip1MoveStraightToIntake = new MoveStraightPIDAction(driveTrain, -800); // negative = backward: at 90 deg the flower is behind the robot (lower y)
//        trip1MoveStraightToIntake.setTimeoutMS(1500);
//        trip1MoveStraightToIntake.setName("trip1MoveStraightToIntake");
//        trip1MoveStraightToIntake.setDependentActions(trip0Shoot);
//        redThreeTipSoloAuto.addAction(trip1MoveStraightToIntake);

        IntakeFullAction trip1IntakeFullAction = new IntakeFullAction(stopper, intake, 1_000_000, IntakeConfig.intakePower);
        trip1IntakeFullAction.setName("trip3IntakeFullAction");
        trip1IntakeFullAction.setDependentActions(trip0Shoot);
        redThreeTipSoloAuto.addAction(trip1IntakeFullAction);
//
//        IPurePursuitAction trip1IntakeSweepGarden = new AdaptivePurePursuitAction(driveTrain);
//        trip1IntakeSweepGarden.setMaxTimeOutMS(3000); // give up if blocked, so the robot never keeps pushing (TODO-measure)
//        trip1IntakeSweepGarden.setName("trip3IntakeGarden");
//        trip1IntakeSweepGarden.addPosition(195, 3309, 90.0);
//        trip1IntakeSweepGarden.addPosition(240, 3200, 95.0);
//        trip1IntakeSweepGarden.addPosition(195, 3309, 90.0);
//        trip1IntakeSweepGarden.setDependentActions(trip1GoToGarden);
//        redThreeTipSoloAuto.addAction(trip1IntakeSweepGarden);

        // Sweep done: stop the intake now, so its timer does not keep it running into the shot (same as FullFlowerAction).
        IntakeStop trip1StopIntake = new IntakeStop(trip1IntakeFullAction);
        trip1StopIntake.setName("trip3StopIntake");
        trip1StopIntake.setDependentActions(trip1GoToGarden);
        redThreeTipSoloAuto.addAction(trip1StopIntake);

//        IPurePursuitAction trip1GoToShoot = new AdaptivePurePursuitAction(driveTrain);
//        trip1GoToShoot.setMaxTimeOutMS(1500); // give up if blocked, so the robot never keeps pushing (TODO-measure)
//        trip1GoToShoot.setName("trip3GoToShoot");
//        trip1GoToShoot.addPosition(941, 3089, 155.9);
//        trip1GoToShoot.setDependentActions(trip1IntakeSweepGarden);
//        redThreeTipSoloAuto.addAction(trip1GoToShoot);

        ShootAllAction trip1GardenShoot = new ShootAllAction(turret, stopper, intake, shooter, shooterRun, turretAutoAlign);
        trip1GardenShoot.setName("trip1GardenShoot");
        trip1GardenShoot.setTargetPoint(ShooterConfig.TARGET_POINT_1); // Cell A
        trip1GardenShoot.setDependentActions(trip1GoToGarden);
        redThreeTipSoloAuto.addAction(trip1GardenShoot);

        //====================================TRIP 2=====================================
        KServoAutoAction flowerLowerTrip1 = new KServoAutoAction(flowerMech.getServo(), FlowerMech.DOWN_POS);
        flowerLowerTrip1.setName("flowerLowerTrip1");
        flowerLowerTrip1.setDependentActions(trip1GardenShoot);
        redThreeTipSoloAuto.addAction(flowerLowerTrip1);

        FullFlowerAction trip2FlowerIntake = new FullFlowerAction(stopper, intake, flowerIntakeMS, IntakeConfig.intakePower, flowerMech, driveTrain, FLOWER_INTAKE_POSITION_2, FLOWER_EXIT_POSITION_2);
        trip2FlowerIntake.setName("trip1FlowerIntake");
        trip2FlowerIntake.setDependentActions(trip1GardenShoot);
//        trip2FlowerIntake.getFlowerLower().setIsDone(true);
        redThreeTipSoloAuto.addAction(trip2FlowerIntake);
//
//        FlowerRaise flowerRaiseTrip1 = new FlowerRaise(flowerMech);
//        flowerRaiseTrip1.setName("flowerRaiseTrip1");
//        flowerRaiseTrip1.setDependentActions(trip1FlowerIntake);
//        redThreeTipSoloAuto.addAction(flowerRaiseTrip1);
//
//        WaitAction waitAfterRaisingFlower = new WaitAction(3000);
//        waitAfterRaisingFlower.setName("waitAfterRaisingFlower");
//        waitAfterRaisingFlower.setDependentActions(flowerRaiseTrip1);
//        redThreeTipSoloAuto.addAction(waitAfterRaisingFlower);

        IPurePursuitAction trip2GoToShoot = new AdaptivePurePursuitAction(driveTrain);
        trip2GoToShoot.setMaxTimeOutMS(4001); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        trip2GoToShoot.setName("trip2GoToShoot");
        trip2GoToShoot.addPosition(812.8, 914.4, 90);
        trip2GoToShoot.addPosition(FLOWER_INTAKE_POSITION_1.getX(), 621, 90);
//        trip2GoToShoot.setPlanStartPosition(new Position(FLOWER_INTAKE_POSITION_2.getX() - FullFlowerAction.PUSH_IN_MM * Math.cos(FLOWER_INTAKE_POSITION_2.getTheta()),
//                FLOWER_INTAKE_POSITION_2.getY() - FullFlowerAction.PUSH_IN_MM * Math.sin(FLOWER_INTAKE_POSITION_2.getTheta()), FLOWER_INTAKE_POSITION_2.getTheta()));
        trip2GoToShoot.setDependentActions(trip2FlowerIntake);
        redThreeTipSoloAuto.addAction(trip2GoToShoot);

        ShootAllAction trip2Shoot = new ShootAllAction(turret, stopper, intake, shooter, shooterRun, turretAutoAlign);
        trip2Shoot.setName("trip2Shoot");
        trip2Shoot.setTargetPoint(ShooterConfig.TARGET_POINT_2); // Cell B
        trip2Shoot.setDependentActions(trip2GoToShoot);
        redThreeTipSoloAuto.addAction(trip2Shoot);
        //END OF TRIP 2

        //===============================TRIP 3===============================

//        FlowerLower flowerLowerTrip2 = new FlowerLower(flowerMech, IntakeConfig.FLOWER_TIMEOUT_MS);
//        flowerLowerTrip2.setName("flowerLowerTrip2");
//        flowerLowerTrip2.setDependentActions(waitAfterRaisingFlower);
//        redThreeTipSoloAuto.addAction(flowerLowerTrip2);

        FullFlowerAction trip3FlowerIntake = new FullFlowerAction(stopper, intake, flowerIntakeMS, IntakeConfig.intakePower, flowerMech, driveTrain, FLOWER_INTAKE_POSITION_1, FLOWER_EXIT_POSITION_1);
        trip3FlowerIntake.setName("trip2FlowerIntake");
//        trip3FlowerIntake.getFlowerLower().setIsDone(true);
        trip3FlowerIntake.setDependentActions(trip2Shoot);
        trip3FlowerIntake.setIntakeTime(800);
        redThreeTipSoloAuto.addAction(trip3FlowerIntake);

//        // TODO-check: added so the robot is at (1037, 397) for this shot, same as trip 1
//        IPurePursuitAction trip2GoToShoot = new AdaptivePurePursuitAction(driveTrain);
//        trip2GoToShoot.setMaxTimeOutMS(1500); // give up if blocked, so the robot never keeps pushing (TODO-measure)
//        trip2GoToShoot.setName("trip2GoToShoot");
//        trip2GoToShoot.addPosition(1052, 543, 100);
//        trip2GoToShoot.setPlanStartPosition(new Position(FLOWER_INTAKE_POSITION_1.getX() - FullFlowerAction.PUSH_IN_MM * Math.cos(FLOWER_INTAKE_POSITION_1.getTheta()),
//                FLOWER_INTAKE_POSITION_1.getY() - FullFlowerAction.PUSH_IN_MM * Math.sin(FLOWER_INTAKE_POSITION_1.getTheta()), FLOWER_INTAKE_POSITION_1.getTheta()));
//        trip2GoToShoot.setDependentActions(trip2FlowerIntake);
//        redThreeTipSoloAuto.addAction(trip2GoToShoot);

//        FlowerRaise flowerRaiseTrip2 = new FlowerRaise(flowerMech);
//        flowerRaiseTrip2.setName("flowerRaiseTrip2");
//        flowerRaiseTrip2.setDependentActions(trip2GoToShoot);
//        redThreeTipSoloAuto.addAction(flowerRaiseTrip2);

        ShootAllAction trip3Shoot = new ShootAllAction(turret, stopper, intake, shooter, shooterRun, turretAutoAlign);
        trip3Shoot.setName("trip3Shoot");
        trip3Shoot.setTargetPoint(ShooterConfig.TARGET_POINT_2); // Cell B
        trip3Shoot.setDependentActions(trip3FlowerIntake);
        redThreeTipSoloAuto.addAction(trip3Shoot);

        WaitAction waitBeforeRaise = new WaitAction(1200);
        waitBeforeRaise.setName("waitBeforeRaise");
        waitBeforeRaise.setDependentActions(trip3Shoot);
        redThreeTipSoloAuto.addAction(waitBeforeRaise);

        KServoAutoAction flowerRaiseTrip3 = new KServoAutoAction(flowerMech.getServo(), FlowerMech.UP_POS);
        flowerRaiseTrip3.setName("flowerRaiseTrip2");
        flowerRaiseTrip3.setDependentActions(waitBeforeRaise); // not delayBeforeStart: it would fight flowerLowerTrip1 over the servo
        redThreeTipSoloAuto.addAction(flowerRaiseTrip3);

        //END OF TRIP 3

        //===============================TRIP 4===============================
//        IPurePursuitAction trip4LookAtBalls = new AdaptivePurePursuitAction(driveTrain);
//        trip4LookAtBalls.setMaxTimeOutMS(2000); // give up if blocked, so the robot never keeps pushing (TODO-measure)
//        trip4LookAtBalls.setName("trip4LookAtBalls");
//        trip4LookAtBalls.addPosition(450, 2321, 221.7);
//        trip4LookAtBalls.setDependentActions(trip3Shoot);
//        redThreeTipSoloAuto.addAction(trip4LookAtBalls);
//
//        // Look from here, then drive to the best balls with the intake on, look again, until full (or nothing left / timeout).
//        VisionPickupAction trip4GrabBalls = new VisionPickupAction(driveTrain, stopper, intake, allianceColor)
//                .setTimeoutMs(visionPickupTimeoutMS);
//        trip4GrabBalls.setName("trip4GrabBalls");
//        trip4GrabBalls.setDependentActions(trip4LookAtBalls);
//        redThreeTipSoloAuto.addAction(trip4GrabBalls);
//
//        IPurePursuitAction trip4GoToShoot = new AdaptivePurePursuitAction(driveTrain);
//        trip4GoToShoot.setMaxTimeOutMS(3000); // give up if blocked, so the robot never keeps pushing (TODO-measure)
//        trip4GoToShoot.setName("trip4GoToShoot");
//        trip4GoToShoot.addPosition(714, 2952, 102.6);
//        trip4GoToShoot.setDependentActions(trip4GrabBalls);
//        redThreeTipSoloAuto.addAction(trip4GoToShoot);
//
//
//        ShootAllAction trip4VisionShoot = new ShootAllAction(turret, stopper, intake, shooter, shooterRun, turretAutoAlign);
//        trip4VisionShoot.setName("trip4VisionShoot");
//        trip4VisionShoot.setTargetPoint(ShooterConfig.TARGET_POINT_1); // Cell A
//        trip4VisionShoot.setDependentActions(trip4GoToShoot);
//        redThreeTipSoloAuto.addAction(trip4VisionShoot);
//
//        ShooterStop shooterStop = new ShooterStop(shooterRun);
//        shooterStop.setName("shooterStop");
//        shooterStop.setDependentActions(trip4VisionShoot);
//        redThreeTipSoloAuto.addAction(shooterStop);
//
//        // Before parking: stop everything except the drivetrain.
//        Action trip4StopEverything = new Action() {
//            @Override
//            protected void update() {
//                stopEverythingButDrive();
//                isDone = true;
//            }
//        };
//        trip4StopEverything.setName("trip4StopEverything");
//        trip4StopEverything.setDependentActions(trip4VisionShoot);
//        redThreeTipSoloAuto.addAction(trip4StopEverything);

        IPurePursuitAction trip4Park = new AdaptivePurePursuitAction(driveTrain);
        trip4Park.setMaxTimeOutMS(3000); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        trip4Park.setName("trip4Park");
        trip4Park.addPosition(PARK_POINT.getX(), PARK_POINT.getY(), 180); // TODO-check: park heading
        trip4Park.setDependentActions(flowerRaiseTrip3);
        redThreeTipSoloAuto.addAction(trip4Park);
        //END OF TRIP 4

        // FORCE PARK: with FORCE_PARK_MS_LEFT left in the match, drop everything and park
        IPurePursuitAction forcePark = new AdaptivePurePursuitAction(driveTrain);
        forcePark.setMaxTimeOutMS(3000); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        forcePark.setName("forcePark");
        forcePark.addPosition(PARK_POINT.getX(), PARK_POINT.getY(), 180); // TODO-check: park heading

        // Pick the start delay. Plan the paths meanwhile: if START is pressed here, a path left unplanned
        // gets planned during the match, and the robot sits still until it is done.
        SetAutoDelayAction setAutoDelayAction = new SetAutoDelayAction(opModeUtilities, gamepad1);
        setAutoDelayAction.setName("setAutoDelayAction");
        while (!setAutoDelayAction.getIsDone() && opModeInInit()) {
            AdaptivePurePursuitAction.runPrecomputeStepForAll();
            // Added before setAutoDelayAction, which sends the telemetry.
            telemetry.addLine(AdaptivePurePursuitAction.precomputeStatus());
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
                KLog.d("RedThreeTipSoloAuto", "force park at " + matchTimer.milliseconds() + " ms");
            }

            if (forceParking) {
                forcePark.updateCheckDone();   // the auto actions are not updated any more
            } else {
                redThreeTipSoloAuto.updateCheckDone();
            }
            KLog.d("Odometry", () -> "Position: " + SharedData.getOdometryWheelIMUPosition());
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
        KLog.d("RedThreeTipSoloAuto", "stopped shooter, intake and turret before parking");
    }
}
