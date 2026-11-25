package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import org.firstinspires.ftc.teamcode.kalipsorobotics.configs.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath.RoundTripAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeStow;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.IPurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;

/**
 * Rigid auto built from gated RoundTripActions (drive with the intake on, then shoot once the drive, flywheel and turret are ready).
 * Order: preload into B, A Flower -> A, B Flower -> B, A-side balls -> A, B-side balls -> B, park.
 * Each trip's path goes to the balls and then on to its launch spot; the points are the earlier shoot-first version's.
 * NOTE: the Flower arm is not lowered on the Flower trips (only the flower's intake side runs).
 */
@Autonomous
public class RedSecondAuto extends KOpMode {
    KActionSet autoSecond;

    private DriveTrain driveTrain;
    private Shooter shooter;
    private Intake intake;
    private Stopper stopper;
    private Turret turret;

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.RED;
        SharedData.setAllianceColor(allianceColor);
        TurretConfig.TURRET_OFFSET_DEG = 0;
    }

    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);

        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000); // Optional: let hardware initialize

        OpModeUtilities.startOdometry(opModeUtilities, odoExecutorService, driveTrain, imuModule,
                new Position(1827 - 172, 172, Math.toRadians(90)));

        autoSecond = new KActionSet();
        intake = new Intake(opModeUtilities);
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);
        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);
        startAprilTagDetection(turret);
        KLog.d("Auto_RedSecond", "event=init");
    }

    private TurretAutoAlign aimAt(Point target) {
        TurretAutoAlign aim = new TurretAutoAlign(opModeUtilities, turret, target);
        aim.setToleranceDeg(5);
        return aim;
    }

    /** Gated trip that shoots into Cell A (aLaunchPoint) or B (bLaunchPoint). intakeSide null = no pickup. */
    private RoundTripAction trip(boolean cellA, IntakeSide intakeSide) {
        Point target = cellA ? ShooterConfig.TARGET_POINT_1 : ShooterConfig.TARGET_POINT_2;
        Point launch = cellA ? FieldConfig.aLaunchPoint : FieldConfig.bLaunchPoint;
        return new RoundTripAction(opModeUtilities, driveTrain, aimAt(target), shooter, stopper, intake, target, launch, 0, intakeSide);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        IntakeSide flowerSide = intake.getSide(ModuleConfig.FLOWER_SIDE);
        IntakeSide sideA = intake.getSide(BeamBreakPosition.IntakeSide.A); // TODO-measure: which side faces the balls on each sweep

        // preload: drive to B's launch spot and shoot
        RoundTripAction trip1 = trip(false, null);
        trip1.getMoveToBall().addPoint(FieldConfig.bLaunchPoint.getX(), FieldConfig.bLaunchPoint.getY(), 90);
        trip1.setName("trip1");
        autoSecond.addAction(trip1);

        // A Flower, then shoot A
        RoundTripAction trip2 = trip(true, flowerSide);
        trip2.getMoveToBall().addPoint(930, FieldConfig.bLaunchPoint.getY(), 90);
        trip2.getMoveToBall().addPoint(FieldConfig.aFlowerPoint.getX() + 500, FieldConfig.aFlowerPoint.getY() - 300, 90);
        trip2.getMoveToBall().addPoint(FieldConfig.aFlowerPoint.getX() + 300, FieldConfig.aFlowerPoint.getY(), 180);
        trip2.getMoveToBall().addPoint(FieldConfig.aLaunchPoint.getX(), FieldConfig.aLaunchPoint.getY(), -90);
        trip2.setName("trip2");
        trip2.setDependentActions(trip1);
        autoSecond.addAction(trip2);

        // B Flower, then shoot B
        RoundTripAction trip3 = trip(false, flowerSide);
        trip3.getMoveToBall().addPoint(1650, 3657.6 - 1300, -90);
        trip3.getMoveToBall().addPoint(1650, 3657.6 - 3000, -90);
        trip3.getMoveToBall().addPoint(FieldConfig.bFlowerPoint.getX(), FieldConfig.bFlowerPoint.getY() + 500, -90);
        trip3.getMoveToBall().addPoint(FieldConfig.bFlowerPoint.getX(), FieldConfig.bFlowerPoint.getY() + 300, -90);
        trip3.getMoveToBall().addPoint(FieldConfig.bLaunchPoint.getX(), FieldConfig.bLaunchPoint.getY(), 90);
        trip3.setName("trip3");
        trip3.setDependentActions(trip2);
        autoSecond.addAction(trip3);

        // todo switch to VisionSweepRoundTripAction when fin
        // A side, then shoot A
        RoundTripAction trip4 = trip(true, sideA);
        trip4.getMoveToBall().addPoint(950, FieldConfig.bLaunchPoint.getY(), 90);
        trip4.getMoveToBall().addPoint(950, 3657.6 - 1200, 90);
        trip4.getMoveToBall().addPoint(950, 3657.6 - 1200, 0);
        trip4.getMoveToBall().addPoint(1600, 3657.6 - 1200, 0);
        trip4.getMoveToBall().addPoint(FieldConfig.aLaunchPoint.getX(), FieldConfig.aLaunchPoint.getY(), -90);
        trip4.setName("trip4");
        trip4.setDependentActions(trip3);
        autoSecond.addAction(trip4);

        // todo switch to VisionSweepRoundTripAction when fin
        // B side, then shoot B
        RoundTripAction trip5 = trip(false, sideA);
        trip5.getMoveToBall().addPoint(1650, 3657.6 - 1300, -90);
        trip5.getMoveToBall().addPoint(1650, 3657.6 - 3000, -90);
        trip5.getMoveToBall().addPoint(1650, 3657.6 - 3000, 180);
        trip5.getMoveToBall().addPoint(1000, 3657.6 - 3000, 180);
        trip5.getMoveToBall().addPoint(FieldConfig.bLaunchPoint.getX(), FieldConfig.bLaunchPoint.getY(), 90);
        trip5.setName("trip5");
        trip5.setDependentActions(trip4);
        autoSecond.addAction(trip5);

        IntakeStow stow = new IntakeStow(intake);
        stow.setName("stow");
        stow.setDependentActions(trip5);
        autoSecond.addAction(stow);

        IPurePursuitAction park = new AdaptivePurePursuitAction(driveTrain);
        park.addPoint(500, 3657.6 - 3350, 180);
        park.addPoint(100, 3657.6 - 3350, 180);
        park.setName("park");
        park.setDependentActions(trip5);
        autoSecond.addAction(park);

        waitForStartPrecomputingPaths();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            intake.update();
            autoSecond.updateCheckDone();
            KLog.d("Odometry", () -> "Position: " + SharedData.getOdometryWheelIMUPosition());
        }
        shooter.stop();
        cleanupRobot();
    }
}
