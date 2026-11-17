package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShootAllAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlignTeleOp;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * Collects (distance, rps, hood, hit) rows for ShooterInterpolationConfig.shooterData.
 * Pattern source: the pre-milestone-2 ShooterDataCollector (ported to the single Shooter, servo Turret,
 * new Intake) plus TeleOp's shoot wiring (ShootAllAction with explicit RPS and hood).
 *
 * Drive to a spot, set RPS and hood on the Dashboard, hold GP1 left bumper to shoot (ShootAllAction feeds
 * the balls), then GP2 A = hit / B = miss writes one row. Each row is already in shooterData's format
 * ("distance, rps, hood,") with the result as a trailing comment, so hit rows paste straight in.
 * GP2 right trigger runs the intake to load balls, GP1 left trigger stops the shooter.
 */
@Config
@TeleOp(name = "ShooterDataCollector", group = "Test")
public class ShooterDataCollector extends KOpMode {
    public static final String TAG = "Test_Shooter";

    public static double targetRPS = 40;
    public static double hoodPosition = 0.2;

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = SharedData.getAllianceColor();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        Turret turret = Turret.getInstance(opModeUtilities);
        Intake intake = new Intake(opModeUtilities);
        Shooter shooter = new Shooter(opModeUtilities);
        Stopper stopper = new Stopper(opModeUtilities);
        sleep(1000);
        OpModeUtilities.runOdometryExecutorService(odoExecutorService,
                Odometry.getInstance(opModeUtilities, driveTrain, imuModule));

        shooterRun = new ShooterRun(opModeUtilities, shooter,
                Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()));
        turretAutoAlignTeleOp = new TurretAutoAlignTeleOp(opModeUtilities, turret, allianceColor);
        turretAutoAlignTeleOp.setToleranceDeg(2);
        KFileWriter rows = new KFileWriter("ShooterDataRows", opModeUtilities);

        stopper.setPosition(ModuleConfig.STOPPER_SERVO_CLOSED_POS);
        intake.lower();
        ShootAllAction shootAllAction = null;
        double shotRps = targetRPS;
        double shotHood = hoodPosition;
        double shotDistance = 0;
        waitForStart();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();

            if (kGamePad2.isRightTriggerPressed()) {
                intake.run(1);
            } else if (shootAllAction == null || shootAllAction.getIsDone()) {
                intake.stop();
            }
            intake.update();

            if (kGamePad1.isLeftBumperPressed() && !isPending(shootAllAction)) {
                turretAutoAlignTeleOp.setTurretRunMode(TurretRunMode.RUN_USING_ODOMETRY);
                shotRps = targetRPS;
                shotHood = hoodPosition;
                shotDistance = shooterRun.getDistanceMM();
                shootAllAction = new ShootAllAction(turret, stopper, intake, shooter, shooterRun,
                        turretAutoAlignTeleOp, targetRPS, hoodPosition);
                shootAllAction.setEnableLimelightZeroing(false);
                shooterRun.setUseOdometry(true);
                setLastShooterAction(shootAllAction);
                KLog.d(TAG, () -> String.format("event=shoot targetRps=%.1f hood=%.4f distanceMm=%.0f",
                        targetRPS, hoodPosition, shooterRun.getDistanceMM()));
            } else if (kGamePad1.isLeftTriggerFirstPressed()) {
                setLastShooterAction(null);
                shooter.stop();
                KLog.d(TAG, "event=stopShooter");
            }

            boolean a = kGamePad2.isButtonAFirstPressed();
            boolean b = kGamePad2.isButtonBFirstPressed();
            if (a || b) {
                boolean hit = a;
                String row = String.format("%.0f, %.1f, %.4f, // %s",
                        shotDistance, shotRps, shotHood, hit ? "hit" : "miss");
                rows.writeLine(row);
                KLog.d(TAG, () -> "event=row hit=" + hit + " row=" + row);
            }

            telemetry.addData("targetRPS", "%.1f", targetRPS);
            telemetry.addData("actualRPS", "%.2f", shooter.getRPS());
            telemetry.addData("hood", "%.4f", hoodPosition);
            telemetry.addData("distanceMm", "%.0f", shooterRun.getDistanceMM());
            telemetry.addData("odometry", SharedData.getOdometryWheelIMUPosition());
            telemetry.addData("ballCount", intake.getBallCount());
            telemetry.update();

            updateActions();
        }

        shooter.stop();
        rows.close();
        cleanupRobot();
    }
}
