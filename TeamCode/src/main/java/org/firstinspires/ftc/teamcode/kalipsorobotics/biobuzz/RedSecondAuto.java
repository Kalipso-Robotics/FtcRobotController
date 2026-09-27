package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath.RoundTripAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.ShooterRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;

@Autonomous
public class RedSecondAuto extends KOpMode {
    KActionSet autoSecond;

    private DriveTrain driveTrain;
    Shooter shooter = null;
    Intake intake = null;
    Stopper stopper = null;
    Turret turret = null;

    TurretAutoAlign turretAutoAlign = null;

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.RED;
        SharedData.setAllianceColor(allianceColor);
        TurretConfig.TICKS_INIT_OFFSET = 0;
    }

    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);

        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000); // Optional: let hardware initialize

        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, -1657, 3486, Math.toRadians(-90));
        OpModeUtilities.runOdometryExecutorService(odoExecutorService, odometry);

        autoSecond = new KActionSet();
        KLog.d("RedAutoDepot-Init", "Creating intake, shooter, stopper modules");
        KLog.d("RedAutoDepot-Init", () -> "opModeUtilities is: " + (opModeUtilities != null ? "NOT NULL" : "NULL"));
        intake = new Intake(opModeUtilities);
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);
        shooterRun = new ShooterRun(opModeUtilities, shooter, 0, ShooterInterpolationConfig.MAX_HOOD);
        shooterRun.setShooterRunMode(ShooterRunMode.STOP);
        KLog.d("RedAutoDepot-Init", () -> "Stopper created: " + (stopper != null ? "SUCCESS" : "NULL"));

        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);
        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, allianceColor);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        // B shoot, go to A flower
        RoundTripAction trip1 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, allianceColor, true);
        trip1.getMoveToShoot().addPoint(FieldConfig.bLaunchPoint.getX(), FieldConfig.bLaunchPoint.getY(), -90);
        trip1.getMoveToBalls().addPoint(-930, FieldConfig.bLaunchPoint.getY(), -90);
//        trip1.getMoveToBalls().addPoint(-930, FieldConfig.aFlowerPoint.getY(), -90);
//        trip1.getMoveToBalls().addPoint(FieldConfig.aFlowerPoint.getX()-500, FieldConfig.aFlowerPoint.getY(), -90);
        trip1.getMoveToBalls().addPoint(FieldConfig.aFlowerPoint.getX()-500, FieldConfig.aFlowerPoint.getY() + 300, -90);
        trip1.getMoveToBalls().addPoint(FieldConfig.aFlowerPoint.getX()-300, FieldConfig.aFlowerPoint.getY(), 0);
//        trip1.getMoveToBalls().addPoint(FieldConfig.aFlowerPoint.getX(), FieldConfig.aFlowerPoint.getY(), 0);
        trip1.setName("trip1");
        autoSecond.addAction(trip1);

        // A shoot, go B flower
        RoundTripAction trip2 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, allianceColor, true);
        trip2.getMoveToShoot().addPoint(FieldConfig.aLaunchPoint.getX(), FieldConfig.aLaunchPoint.getY(), 90);
        trip2.getMoveToBalls().addPoint(-1650, 1300, 90);
        trip2.getMoveToBalls().addPoint(-1650, 3000, 90);
        trip2.getMoveToBalls().addPoint(FieldConfig.bFlowerPoint.getX(), FieldConfig.bFlowerPoint.getY() - 500, 90);
        trip2.getMoveToBalls().addPoint(FieldConfig.bFlowerPoint.getX(), FieldConfig.bFlowerPoint.getY() - 300, 90);
        trip2.setName("trip2");
        trip2.setDependentActions(trip1);
        autoSecond.addAction(trip2);

        // B shoot, go to A side
        RoundTripAction trip3 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, allianceColor, true);
//        trip3.getMoveToShoot().addPoint(FieldConfig.bFlowerPoint.getX(), FieldConfig.bFlowerPoint.getY() - 500, 90);
//        trip3.getMoveToShoot().addPoint(FieldConfig.bFlowerPoint.getX(), FieldConfig.bFlowerPoint.getY() - 500, -90);
        trip3.getMoveToShoot().addPoint(FieldConfig.bLaunchPoint.getX(), FieldConfig.bLaunchPoint.getY(), -90);
        trip3.getMoveToBalls().addPoint(-950, FieldConfig.bLaunchPoint.getY(), -90);
        trip3.getMoveToBalls().addPoint(-950, 1200, -90);
        trip3.getMoveToBalls().addPoint(-950, 1200, -180);
        trip3.getMoveToBalls().addPoint(-1600, 1200, -180);
//        trip3.getMoveToBalls().addPoint(0, 0, 0);
        trip3.setName("trip3");
        trip3.setDependentActions(trip2);
        autoSecond.addAction(trip3);

        //A shoot, go to B side
        RoundTripAction trip4 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, allianceColor, true);
        trip4.getMoveToShoot().addPoint(FieldConfig.aLaunchPoint.getX(), FieldConfig.aLaunchPoint.getY(), 90);
        trip4.getMoveToBalls().addPoint(-1650, 1300, 90);
        trip4.getMoveToBalls().addPoint(-1650, 3000, 90);
        trip4.getMoveToBalls().addPoint(-1650, 3000, 0);
        trip4.getMoveToBalls().addPoint(-1000, 3000, 0);
        trip4.setName("trip4");
        trip4.setDependentActions(trip3);
        autoSecond.addAction(trip4);

        // B shoot, go to A side
        RoundTripAction trip5 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, allianceColor, true);
        trip5.getMoveToShoot().addPoint(FieldConfig.bLaunchPoint.getX(), FieldConfig.bLaunchPoint.getY(), -90);
        trip5.setName("trip5");
        trip5.setDependentActions(trip4);
        autoSecond.addAction(trip5);

        waitForStartPrecomputingPaths();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            autoSecond.updateCheckDone();
            KLog.d("Odometry", () -> "Position: " + SharedData.getOdometryWheelIMUPosition());
        }
        cleanupRobot();
    }
}
