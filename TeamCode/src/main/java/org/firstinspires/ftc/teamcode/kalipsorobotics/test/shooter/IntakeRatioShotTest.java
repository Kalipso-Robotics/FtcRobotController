package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.FieldOrientedDriveAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShootAllAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.configs.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.ResetOdometryToPos;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.ShooterRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import java.util.Map;
import java.util.TreeMap;

/**
 * Shot consistency vs. the speed ratio of the two intake sides during ShootAllAction's feed.
 * Pattern source: RPSPredictionDataCollector (same setup, shot flow and cancel), trimmed to the ratio question.
 *
 * Side A feeds at IntakeConfig.FEED_RPS, side B at FEED_RPS * IntakeConfig.FEED_SIDE_B_RATIO (IntakeRun). Both are
 * changed live here and put back at the end: copy the winning values into IntakeConfig by hand.
 * After each shot, mark it hit (B) or miss (LT + B): every marked shot is a CSV row, hits AND misses, so the file
 * gives a hit rate per ratio. Telemetry keeps a running hits/shots tally per ratio.
 * Output: RobotLogs/IntakeRatioShot_*.csv
 *
 * Controls (GP1):
 *   sticks              field-centric drive        turret auto-aims at the Hive
 *   dpad up / down      shooter target RPS +/- RPS_STEP
 *   dpad right / left   hood +/- HOOD_STEP
 *   Y / A               side B ratio +/- RATIO_STEP (B RPS = A RPS * ratio)
 *   LT hold + Y / A     feed RPS (side A) +/- FEED_RPS_STEP
 *   LT hold + dpad down ratio back to 1
 *   X                   shooter on/off
 *   RT hold             intake (both sides) to load balls
 *   LB                  shoot (ShootAllAction); LB again cancels
 *   B / LT hold + B     mark the last shot HIT / MISS (writes the row)
 *   RB                  rezero: robot in the loading-zone corner facing +x
 */
@Config
@TeleOp(name = "Intake Ratio Shot Test", group = "Test")
public class IntakeRatioShotTest extends KOpMode {
    public static final String TAG = "Test_IntakeRatio";
    public static double TURRET_TOLERANCE_DEG = 2;
    public static double RATIO_STEP = 0.05;
    public static double MIN_RATIO = 0.2;
    public static double MAX_RATIO = 3.0;
    private static final double RPS_STEP = 1;
    private static final double HOOD_STEP = 0.005;
    private static final double FEED_RPS_STEP = 1;

    private static final String[] CONTROLS = {
            "dpad up/down: shooter RPS +/-" + RPS_STEP + "   dpad right/left: hood +/-" + HOOD_STEP,
            "Y/A: side B ratio +/-" + RATIO_STEP + "   LT+Y/A: feed RPS +/-" + FEED_RPS_STEP + "   LT+dpad down: ratio 1",
            "X: shooter on/off   RT hold: intake   LB: shoot / cancel",
            "B: mark HIT   LT+B: mark MISS   RB: rezero corner",
    };

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        Shooter shooter = new Shooter(opModeUtilities);
        Intake intake = new Intake(new IntakeModule(opModeUtilities, IntakeSide.FRONT, IntakeConfig.SIDE_A),
                new IntakeModule(opModeUtilities, IntakeSide.BACK, IntakeConfig.SIDE_B), null);
        IntakeModule sideA = intake.getModule(IntakeSide.FRONT);
        IntakeModule sideB = intake.getModule(IntakeSide.BACK);
        Stopper stopper = new Stopper(opModeUtilities);
        Turret turret = Turret.getInstance(opModeUtilities);
        FieldOrientedDriveAction driveAction = new FieldOrientedDriveAction(driveTrain, imuModule);
        OpModeUtilities.startOdometry(opModeUtilities, odoExecutorService, driveTrain, imuModule, null);

        double feedRpsAtStart = IntakeConfig.FEED_RPS;
        double ratioAtStart = IntakeConfig.FEED_SIDE_B_RATIO;
        turretAutoAlignTeleOp = new TurretAutoAlign(opModeUtilities, turret);
        turretAutoAlignTeleOp.setTurretRunMode(TurretRunMode.RUN_USING_ODOMETRY);
        turretAutoAlignTeleOp.setToleranceDeg(TURRET_TOLERANCE_DEG);
        stopper.setPosition(ModuleConfig.STOPPER_SERVO_CLOSED_POS);

        KFileWriter fileWriter = new KFileWriter("IntakeRatioShot", opModeUtilities);
        fileWriter.writeLine("Result,RatioB,FeedRPSA,FeedRPSB,ShooterRPSAtOpen,AvgRPSA,AvgRPSB,TargetRPS,MinShooterRPS,"
                + "HoodPosition,DistanceToTargetMM,Voltage,IntakeTimedOut");
        flush(fileWriter);

        double targetRPS = 38;
        double hoodPosition = 0.25;
        shooter.getHood().setPosition(hoodPosition);
        ShooterRun rpsLoop = new ShooterRun(opModeUtilities, shooter, 0, hoodPosition);
        rpsLoop.stop();
        shooterRun = rpsLoop; // updateActions() ticks it

        boolean shooterOn = false;
        ShootAllAction shot = null;
        boolean wasShooting = false;
        String message = "";
        String lastShot = null;   // finished shot waiting for HIT / MISS
        String lastRatioKey = null;
        // Feed stats for the shot in progress (from the stopper opening until the shot ends)
        String openRow = null;
        double minShooterRPS = 0;
        double sumRpsA = 0, sumRpsB = 0;
        int feedSamples = 0;
        Map<String, int[]> tally = new TreeMap<>(); // ratio -> {hits, shots}

        for (String line : CONTROLS) {
            telemetry.addLine(line);
        }
        telemetry.addLine("Press PLAY to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            Point hiveTarget = SharedData.getTargetPoint();
            double distanceMM = ShooterRun.getDistanceToTargetFromCurrentPos(hiveTarget);

            boolean lt = gamepad1.left_trigger > 0.1;
            boolean dpadUp = kGamePad1.isDpadUpFirstPressed();
            boolean dpadDown = kGamePad1.isDpadDownFirstPressed();
            boolean dpadLeft = kGamePad1.isDpadLeftFirstPressed();
            boolean dpadRight = kGamePad1.isDpadRightFirstPressed();
            boolean buttonA = kGamePad1.isButtonAFirstPressed();
            boolean buttonB = kGamePad1.isButtonBFirstPressed();
            boolean buttonX = kGamePad1.isButtonXFirstPressed();
            boolean buttonY = kGamePad1.isButtonYFirstPressed();

            if (kGamePad1.isRightBumperFirstPressed()) {
                setLastZeroAction(new ResetOdometryToPos(new Position(FieldConfig.CORNER_ZERO_POSITION)));
                message = "Rezeroed to corner " + FieldConfig.CORNER_ZERO_POSITION;
                KLog.d(TAG, () -> "event=rezero to=" + FieldConfig.CORNER_ZERO_POSITION);
            }

            // Ratio and feed RPS: read live by IntakeRun every loop, so a change mid-shot applies immediately
            if (!lt && (buttonY || buttonA)) {
                IntakeConfig.FEED_SIDE_B_RATIO = MathFunctions.clamp(
                        round2(IntakeConfig.FEED_SIDE_B_RATIO + (buttonY ? RATIO_STEP : -RATIO_STEP)), MIN_RATIO, MAX_RATIO);
                KLog.d(TAG, () -> "event=ratioSet ratioB=" + IntakeConfig.FEED_SIDE_B_RATIO);
            }
            if (lt && dpadDown) {
                IntakeConfig.FEED_SIDE_B_RATIO = 1.0;
                KLog.d(TAG, "event=ratioSet ratioB=1.0");
            }
            if (lt && (buttonY || buttonA)) {
                IntakeConfig.FEED_RPS = Math.max(FEED_RPS_STEP, IntakeConfig.FEED_RPS + (buttonY ? FEED_RPS_STEP : -FEED_RPS_STEP));
                KLog.d(TAG, () -> "event=feedRpsSet rps=" + IntakeConfig.FEED_RPS);
            }

            double rpsBefore = targetRPS;
            if (!lt && dpadUp) {
                targetRPS += RPS_STEP;
            }
            if (!lt && dpadDown) {
                targetRPS -= RPS_STEP;
            }
            targetRPS = MathFunctions.clamp(targetRPS, 0, ShooterInterpolationConfig.MAX_RPS);
            if (targetRPS != rpsBefore) {
                double r = targetRPS;
                KLog.d(TAG, () -> "event=targetRpsSet targetRps=" + r);
            }
            if (!lt && dpadRight) {
                hoodPosition += HOOD_STEP;
            }
            if (!lt && dpadLeft) {
                hoodPosition -= HOOD_STEP;
            }
            hoodPosition = MathFunctions.clamp(hoodPosition, ShooterInterpolationConfig.MIN_HOOD, ShooterInterpolationConfig.MAX_HOOD);
            if (!shooterOn) {
                shooter.getHood().setPosition(hoodPosition); // when on, ShooterRun drives the hood
            }

            if (buttonX) {
                shooterOn = !shooterOn;
                if (!shooterOn && isPending(shot)) {
                    shot = cancelShot(shot, stopper, intake);
                    message = "Shot cancelled: shooter off";
                }
                boolean on = shooterOn;
                KLog.d(TAG, () -> "event=shooterToggle on=" + on);
            }

            driveAction.move(gamepad1);
            updateActions();

            boolean shooting = isPending(shot);
            // Feed stats: start when the stopper starts opening, sample every loop until the shot ends
            if (shooting && shot.getOpenStopper().getHasStarted()) {
                if (openRow == null) {
                    openRow = String.format("%.2f,%.2f,%.2f,%.2f",
                            IntakeConfig.FEED_SIDE_B_RATIO, shot.getIntakeRun().getTargetRPSSideA(),
                            shot.getIntakeRun().getTargetRPSSideB(), shooter.getRPS());
                    lastRatioKey = String.format("%.2f", IntakeConfig.FEED_SIDE_B_RATIO);
                    minShooterRPS = shooter.getRPS();
                    sumRpsA = 0;
                    sumRpsB = 0;
                    feedSamples = 0;
                    KLog.d(TAG, () -> "event=feedStart ratioB=" + IntakeConfig.FEED_SIDE_B_RATIO + " shooterRps=" + shooter.getRPS());
                }
                minShooterRPS = Math.min(minShooterRPS, shooter.getRPS());
                sumRpsA += sideA.getRPS();
                sumRpsB += sideB.getRPS();
                feedSamples++;
            }
            if (wasShooting && !shooting) {
                if (openRow == null) {
                    message = "Not fed: shooter / turret / intake not ready in time, nothing recorded";
                    KLog.d(TAG, "event=shotAborted");
                } else {
                    int n = Math.max(1, feedSamples);
                    lastShot = String.format("%s,%.2f,%.2f,%.2f,%.2f,%.4f,%.0f,%.2f,%d",
                            openRow, sumRpsA / n, sumRpsB / n, targetRPS, minShooterRPS,
                            hoodPosition, distanceMM, SharedData.getVoltage(), shot.intakeTimedOut() ? 1 : 0);
                    message = "Shot done: B = HIT, LT+B = MISS";
                    String s = lastShot;
                    KLog.d(TAG, () -> "event=shotDone row=" + s);
                }
                openRow = null;
                shot = null;
                turretAutoAlignTeleOp.setTurretRunMode(TurretRunMode.RUN_USING_ODOMETRY); // ShootAllAction stops it once the stopper opens
            }
            wasShooting = shooting;

            if (kGamePad1.isLeftBumperFirstPressed()) {
                if (shooting) {
                    shot = cancelShot(shot, stopper, intake);
                    shooting = false;
                    wasShooting = false;
                    openRow = null;
                    message = "Shot cancelled";
                } else if (!shooterOn) {
                    message = "Shooter is off: press X first";
                } else {
                    shot = new ShootAllAction(turret, stopper, intake, shooter, rpsLoop, turretAutoAlignTeleOp, targetRPS, hoodPosition);
                    shot.setEnableLimelightZeroing(false);
                    shot.setName("ratioTestShot");
                    setLastShooterAction(shot);
                    shooting = true;
                    wasShooting = true;
                    lastShot = null;
                    message = "";
                }
            }

            if (buttonB) {
                if (shooting) {
                    message = "Shot still running: mark it after it ends";
                } else if (lastShot == null) {
                    message = "Nothing to mark: shoot with LB first";
                } else {
                    boolean hit = !lt;
                    String row = (hit ? "HIT," : "MISS,") + lastShot;
                    fileWriter.writeLine(row);
                    flush(fileWriter);
                    int[] t = tally.get(lastRatioKey);
                    if (t == null) {
                        t = new int[2];
                        tally.put(lastRatioKey, t);
                    }
                    if (hit) {
                        t[0]++;
                    }
                    t[1]++;
                    message = "SAVED " + row;
                    KLog.d(TAG, () -> "event=saved row=" + row);
                    lastShot = null;
                }
            }

            if (!shooting) {
                if (gamepad1.right_trigger > 0.1) {
                    intake.lower();
                    intake.run(1.0);
                } else {
                    intake.stop();
                }
            }

            if (!shooterOn) {
                if (rpsLoop.getShooterRunMode() != ShooterRunMode.STOP) {
                    rpsLoop.stop();
                }
            } else if (!shooting) { // a running ShootAllAction sets the mode itself
                rpsLoop.setTargetRPS(targetRPS);
                rpsLoop.setTargetHoodPosition(hoodPosition);
                if (rpsLoop.getShooterRunMode() != ShooterRunMode.SHOOT_USING_TARGET_RPS_HOOD) {
                    rpsLoop.setShooterRunMode(ShooterRunMode.SHOOT_USING_TARGET_RPS_HOOD);
                }
            }

            for (String line : CONTROLS) {
                telemetry.addLine(line);
            }
            if (!message.isEmpty()) {
                telemetry.addLine(message);
            }
            telemetry.addData("Side B ratio", "%.2f   (A %.2f RPS, B %.2f RPS)", IntakeConfig.FEED_SIDE_B_RATIO,
                    IntakeConfig.FEED_RPS, IntakeConfig.FEED_RPS * IntakeConfig.FEED_SIDE_B_RATIO);
            if (!shooting) {
                telemetry.addData("Shot", lastShot != null ? "done: mark HIT (B) / MISS (LT+B)" : "idle");
            } else if (shot.getOpenStopper().getHasStarted()) {
                telemetry.addData("Shot", "FEEDING");
            } else {
                telemetry.addData("Shot", "waiting: shooter %s  turret %s  intake %s",
                        shot.getShooterReady().getIsDone() ? "READY" : "no", turretAutoAlignTeleOp.isWithinRange() ? "READY" : "no",
                        shot.getIntakeReady().getIsDone() ? (shot.intakeTimedOut() ? "TIMED OUT" : "READY") : "no");
            }
            telemetry.addData("Intake actual RPS A / B", "%.2f / %.2f", sideA.getRPS(), sideB.getRPS());
            telemetry.addData("Intake power A / B", "%.2f / %.2f%s", sideA.getPower(), sideB.getPower(),
                    Math.max(sideA.getPower(), sideB.getPower()) >= 0.99 ? "  MAXED: a side can't reach its target" : "");
            telemetry.addData("Shooter on / target / actual RPS", "%b / %.1f / %.2f", shooterOn, targetRPS, shooter.getRPS());
            telemetry.addData("Hood", "%.4f", hoodPosition);
            telemetry.addData("Turret to target", "%.0f mm", distanceMM);
            for (Map.Entry<String, int[]> e : tally.entrySet()) {
                int[] t = e.getValue();
                telemetry.addData("ratio " + e.getKey(), "%d / %d hits (%.0f%%)", t[0], t[1], 100.0 * t[0] / t[1]);
            }
            telemetry.update();
        }

        // static: don't leak the test values into TeleOp. Copy the winners into IntakeConfig by hand.
        KLog.d(TAG, () -> "event=end finalRatioB=" + IntakeConfig.FEED_SIDE_B_RATIO + " finalFeedRps=" + IntakeConfig.FEED_RPS);
        IntakeConfig.FEED_RPS = feedRpsAtStart;
        IntakeConfig.FEED_SIDE_B_RATIO = ratioAtStart;
        if (isPending(shot)) {
            cancelShot(shot, stopper, intake);
        }
        rpsLoop.stop();
        shooterRun = null;
        shooter.stop();
        intake.stop();
        driveTrain.setPower(0);
        fileWriter.close();
        cleanupRobot();
    }

    /** Same as RPSPredictionDataCollector.cancelShot: a cancel can land mid-feed. Returns null. */
    private ShootAllAction cancelShot(ShootAllAction shot, Stopper stopper, Intake intake) {
        shot.setIsDone(true);
        setLastShooterAction(null);
        stopper.setPosition(ModuleConfig.STOPPER_SERVO_CLOSED_POS);
        intake.stop();
        turretAutoAlignTeleOp.setTurretRunMode(TurretRunMode.RUN_USING_ODOMETRY);
        KLog.d(TAG, "event=shotCancelled");
        return null;
    }

    private static double round2(double v) {
        return Math.round(v * 100) / 100.0; // keeps 0.05 steps from drifting (0.9500000001) so the tally keys match
    }

    private static void flush(KFileWriter writer) {
        try {
            writer.flush();
        } catch (java.io.IOException e) {
            KLog.e(TAG, "event=flushFailed", e);
        }
    }
}
