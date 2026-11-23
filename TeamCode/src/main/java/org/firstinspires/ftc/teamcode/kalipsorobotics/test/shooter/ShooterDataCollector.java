package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShootAllAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
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
        OpModeUtilities.startOdometry(opModeUtilities, odoExecutorService, driveTrain, imuModule, null); // keeps the pose from auto, else (0,0,0)

        // Same goal as TeleOp: the Hive Cell on the robot's end of the Hive, so logged distances match TeleOp's.
        Point hiveTarget = SharedData.getTargetPoint();
        shooterRun = new ShooterRun(opModeUtilities, shooter, hiveTarget);
        turretAutoAlignTeleOp = new TurretAutoAlign(opModeUtilities, turret); // follows SharedData.getTargetPoint()
        turretAutoAlignTeleOp.setTurretRunMode(TurretRunMode.RUN_USING_ODOMETRY); // aims the whole time, like TeleOp
        turretAutoAlignTeleOp.setToleranceDeg(2);
        KFileWriter rows = new KFileWriter("ShooterDataRows", opModeUtilities);
        rows.writeLine("// distanceMM, rps, hood, // hit|miss measuredRps=<flywheel RPS at the shot>   paste only the // hit rows into ShooterInterpolationConfig.shooterData");
        flush(rows);

        stopper.setPosition(ModuleConfig.STOPPER_SERVO_CLOSED_POS);
        intake.lower();
        ShootAllAction shootAllAction = null;
        double shotRps = targetRPS;
        double shotHood = hoodPosition;
        double shotDistance = 0;
        double shotMeasuredRps = 0;
        int hits = 0;
        int misses = 0;
        String lastRow = "none yet";
        String message = "";
        waitForStart();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();

            if (kGamePad2.isRightTriggerPressed()) {
                intake.run(1);
            } else if (shootAllAction == null || shootAllAction.getIsDone()) {
                intake.stop();
            }
            intake.update();

            // Same rule as TeleOp: the Cell on the robot's end of the Hive, refreshed as the robot moves.
            hiveTarget = SharedData.getTargetPoint();
            shooterRun.setTargetPoint(hiveTarget);
            if (isPending(shootAllAction)) {
                shotMeasuredRps = shooter.getRPS(); // the feed is the end of the action: the last running loop is the shot
            } else {
                turretAutoAlignTeleOp.setTurretRunMode(TurretRunMode.RUN_USING_ODOMETRY); // ShootAllAction stops it at the feed
            }

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
            if ((a || b) && shootAllAction == null) {
                message = "No shot yet: shoot with GP1 LB first, then mark it";
                KLog.d(TAG, "event=rowRefused reason=noShot");
            } else if (a || b) {
                boolean hit = a;
                String row = String.format("%.0f, %.1f, %.4f, // %s measuredRps=%.1f",
                        shotDistance, shotRps, shotHood, hit ? "hit" : "miss", shotMeasuredRps);
                rows.writeLine(row);
                flush(rows);
                if (hit) {
                    hits++;
                } else {
                    misses++;
                }
                lastRow = row;
                message = "";
                KLog.d(TAG, () -> "event=row hit=" + hit + " row=" + row);
            }

            telemetry.addLine("GP1 LB = shoot   GP1 LT = stop shooter   GP2 RT = intake   GP2 A = hit   GP2 B = miss");
            telemetry.addLine("Set targetRPS and hoodPosition live in Dashboard (ShooterDataCollector)");
            if (!message.isEmpty()) {
                telemetry.addLine(message);
            }
            telemetry.addData("last row saved", lastRow);
            telemetry.addData("hits / misses", "%d / %d", hits, misses);
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

    /** Write the row to the file now, so a crash or a stop does not lose it. */
    private static void flush(KFileWriter rows) {
        try {
            rows.flush();
        } catch (java.io.IOException e) {
            KLog.e(TAG, "event=flushFailed", e);
        }
    }
}
