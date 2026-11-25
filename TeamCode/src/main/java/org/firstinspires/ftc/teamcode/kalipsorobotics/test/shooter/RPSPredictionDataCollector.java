package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.FieldOrientedDriveAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher.PushBall;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.configs.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.ResetOdometryToPos;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * Open-loop check: what RPS a given motor power gives, at the battery voltage, hood and distance at the time.
 * Output: RobotLogs/RPSPredictionData_*.csv with CurrentRPS,CurrentPower,CurrentVoltage,HoodPosition,DistanceToTargetMM,
 * TargetX,TargetY (the Hive Cell), TurretOffsetDeg, AimX,AimY (the point aimed at: the Cell swung around the pivot by the offset).
 * Distance is from the turret pivot (odometry) to the Hive Cell on the robot's end (SharedData.getTargetPoint).
 * Pattern source: this file's earlier LinearOpMode version, ported to KOpMode + startOdometry.
 *
 * Turret aims at the Hive target the whole time (TurretAutoAlign, ticked by updateActions()).
 * Intake: both sides run while RT is held (not while PushBall is feeding).
 * Consistent shots: LB arms a shot; PushBall (the same feed TeleOp uses) starts only once the turret is aligned
 * and the flywheel has held the same power for SETTLE_MS (after power changes, shooter on, and the last feed).
 * The row's RPS is read at that moment, i.e. the steady RPS the first ball leaves at.
 *
 * Controls (GP1):
 *   sticks            field-centric drive (FieldOrientedDriveAction)
 *   dpad up / down    shooter power +/- POWER_STEP_BIG        A / B    power +/- POWER_STEP_SMALL
 *   dpad right / left hood +/- HOOD_STEP (clamped MIN_HOOD..MAX_HOOD)
 *   LT (hold) + A / B hood +/- HOOD_STEP_FINE
 *   X                 toggle the shooter on/off at the set power
 *   RT (hold)         run the intake (both sides)
 *   LB                arm a shot (press again to cancel); PushBall fires when ready and records RPS/power/voltage/hood/distance
 *   Y                 save the last recorded shot to the file (save hits only)
 *   RB                rezero: robot pushed into the loading-zone corner facing +x -> FieldConfig.CORNER_ZERO_POSE
 *   Back              rezero: robot pushed into the far (high x, low y) corner facing +x -> FieldConfig.FAR_CORNER_ZERO_POSE
 *   LT (hold) + dpad left / right   turret offset -/+ TURRET_OFFSET_STEP_DEG (TurretConfig.TURRET_OFFSET_DEG; 0 at start and at the end)
 *   LT (hold) + dpad down           turret offset back to 0
 * All on GP1: Start+A / Start+B to swap gamepads would change the power. The offset only turns the turret: the power is
 * the set one and DistanceToTargetMM stays pivot to Cell.
 *
 * Turret debug: KLog tag Test_Shooter, "event=turret" every TURRET_LOG_MS: pose, target, field aim, turret deg,
 * servo position. Positive turretDeg must swing the turret to the robot's RIGHT (clockwise from above).
 */
@Config
@TeleOp(name = "RPS Prediction Data Collector", group = "Test")
public class RPSPredictionDataCollector extends KOpMode {
    public static final String TAG = "Test_Shooter";
    /** Flywheel must hold one power this long (and this long after a feed) before PushBall fires. TODO-measure */
    public static double SETTLE_MS = 1500;
    public static double TURRET_TOLERANCE_DEG = 2;
    private static final double POWER_STEP_BIG = 0.05;
    private static final double POWER_STEP_SMALL = 0.01;
    private static final double HOOD_STEP = 0.005;
    private static final double HOOD_STEP_FINE = 0.001;
    private static final double TURRET_OFFSET_STEP_DEG = 1;
    private static final long TURRET_LOG_MS = 250;

    private static final String[] CONTROLS = {
            "sticks: field-centric drive   turret auto-aims at the Hive",
            "dpad up/down: power +/-" + POWER_STEP_BIG + "   A/B: power +/-" + POWER_STEP_SMALL,
            "dpad right/left: hood +/-" + HOOD_STEP + "   LT hold + A/B: hood +/-" + HOOD_STEP_FINE,
            "X: shooter on/off   RT hold: intake",
            "LB: arm/cancel shot (PushBall fires when aligned + settled)   Y: save last shot (hits only)",
            "RB: rezero odometry (loading-zone corner, facing +x)   Back: far corner, facing +x",
            "LT hold + dpad left/right: turret offset -/+" + TURRET_OFFSET_STEP_DEG + " deg   LT hold + dpad down: offset 0",
    };

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        Shooter shooter = new Shooter(opModeUtilities);
        // flowerPivot not plugged in yet: no flower; this OpMode only runs the intake motors
        Intake intake = new Intake(new IntakeSide(opModeUtilities, BeamBreakPosition.IntakeSide.A),
                new IntakeSide(opModeUtilities, BeamBreakPosition.IntakeSide.B), null);
        Stopper stopper = new Stopper(opModeUtilities);
        Turret turret = Turret.getInstance(opModeUtilities);
        FieldOrientedDriveAction driveAction = new FieldOrientedDriveAction(driveTrain, imuModule);
        OpModeUtilities.startOdometry(opModeUtilities, odoExecutorService, driveTrain, imuModule, null); // keeps the pose from auto, else the corner

        TurretConfig.TURRET_OFFSET_DEG = 0;
        turretAutoAlignTeleOp = new TurretAutoAlign(opModeUtilities, turret); // follows SharedData.getTargetPoint(); updateActions() ticks it
        turretAutoAlignTeleOp.setTurretRunMode(TurretRunMode.RUN_USING_ODOMETRY);
        turretAutoAlignTeleOp.setToleranceDeg(TURRET_TOLERANCE_DEG);
        stopper.setPosition(ModuleConfig.STOPPER_SERVO_CLOSED_POS);
        KLog.d(TAG, () -> String.format("event=turretConfig degPerServoDeg=%.4f servoRangeDeg=%.1f zeroPos=%.3f dir=%d limitsDeg=%.1f..%.1f pivot=%s startPose=%s",
                TurretConfig.TURRET_DEG_PER_SERVO_DEG, TurretConfig.SERVO_RANGE_DEG, TurretConfig.SERVO_ZERO_POS,
                TurretConfig.SERVO_DIRECTION, TurretConfig.MIN_TURRET_DEG, TurretConfig.MAX_TURRET_DEG, TurretConfig.ROBOT_REL_TURRET_POINT,
                SharedData.getOdometryWheelIMUPosition()));
        long lastTurretLogMs = 0;

        KFileWriter fileWriter = new KFileWriter("RPSPredictionData", opModeUtilities);
        fileWriter.writeLine("CurrentRPS,CurrentPower,CurrentVoltage,HoodPosition,DistanceToTargetMM,TargetX,TargetY,TurretOffsetDeg,AimX,AimY");
        flush(fileWriter);

        double shooterPower = 0;
        double hoodPosition = ShooterInterpolationConfig.MIN_HOOD;
        shooter.getHood().setPosition(hoodPosition);
        Point hiveTarget = null;
        String lastShot = null;
        String message = "";
        boolean shooterOn = false;
        boolean shotArmed = false;
        PushBall pushBall = null;
        boolean wasFeeding = false;
        ElapsedTime settleTimer = new ElapsedTime();

        for (String line : CONTROLS) {
            telemetry.addLine(line);
        }
        telemetry.addLine("Press PLAY to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            hiveTarget = SharedData.getTargetPoint();
            double distanceMM = ShooterRun.getDistanceToTargetFromCurrentPos(hiveTarget); // turret pivot to Cell, same call that reads the table
            if (kGamePad1.isRightBumperFirstPressed()) {
                setLastZeroAction(new ResetOdometryToPos(new Position(FieldConfig.CORNER_ZERO_POSE))); // updateActions() applies it
                message = "Rezeroed to corner " + FieldConfig.CORNER_ZERO_POSE;
                KLog.d(TAG, () -> "event=rezero from=" + SharedData.getOdometryWheelIMUPosition() + " to=" + FieldConfig.CORNER_ZERO_POSE);
            }
            if (kGamePad1.isBackButtonFirstPressed()) {
                setLastZeroAction(new ResetOdometryToPos(new Position(FieldConfig.FAR_CORNER_ZERO_POSE)));
                message = "Rezeroed to far corner " + FieldConfig.FAR_CORNER_ZERO_POSE;
                KLog.d(TAG, () -> "event=rezero from=" + SharedData.getOdometryWheelIMUPosition() + " to=" + FieldConfig.FAR_CORNER_ZERO_POSE);
            }
            // Read each once per loop: FirstPressed clears itself. LT held = the dpad trims the turret and A/B step the hood finely; power and the coarse hood step are off.
            boolean trimTurret = gamepad1.left_trigger > 0.1;
            boolean dpadUp = kGamePad1.isDpadUpFirstPressed();
            boolean dpadDown = kGamePad1.isDpadDownFirstPressed();
            boolean dpadLeft = kGamePad1.isDpadLeftFirstPressed();
            boolean dpadRight = kGamePad1.isDpadRightFirstPressed();
            boolean buttonA = kGamePad1.isButtonAFirstPressed();
            boolean buttonB = kGamePad1.isButtonBFirstPressed();
            double offsetBefore = TurretConfig.TURRET_OFFSET_DEG;
            if (trimTurret && dpadLeft) {
                TurretConfig.TURRET_OFFSET_DEG -= TURRET_OFFSET_STEP_DEG;
            }
            if (trimTurret && dpadRight) {
                TurretConfig.TURRET_OFFSET_DEG += TURRET_OFFSET_STEP_DEG;
            }
            if (trimTurret && dpadDown) {
                TurretConfig.TURRET_OFFSET_DEG = 0;
            }
            if (TurretConfig.TURRET_OFFSET_DEG != offsetBefore) {
                KLog.d(TAG, () -> "event=turretOffsetSet offsetDeg=" + TurretConfig.TURRET_OFFSET_DEG);
            }
            driveAction.move(gamepad1);
            updateActions();

            long nowMs = System.currentTimeMillis();
            if (nowMs - lastTurretLogMs >= TURRET_LOG_MS) {
                lastTurretLogMs = nowMs;
                Position pose = SharedData.getOdometryWheelIMUPosition();
                Point launcher = Turret.launcherPoint(pose);
                Point target = turretAutoAlignTeleOp.getTargetPoint();
                double fieldAimDeg = Math.toDegrees(Math.atan2(target.getY() - launcher.getY(), target.getX() - launcher.getX()));
                KLog.d(TAG, () -> String.format("event=turret x=%.0f y=%.0f headingDeg=%.1f target=(%.0f,%.0f) fieldAimDeg=%.1f wantDeg=%.1f turretDeg=%.1f servoPos=%.4f clamped=%b moving=%b aligned=%b",
                        pose.getX(), pose.getY(), Math.toDegrees(pose.getTheta()), target.getX(), target.getY(), fieldAimDeg,
                        Math.toDegrees(turretAutoAlignTeleOp.getTargetAngleRad()), turret.getAngleDeg(),
                        Turret.servoPosFromTurretDeg(turret.getAngleDeg()), turret.isClamped(), turret.isMoving(),
                        turretAutoAlignTeleOp.isWithinRange()));
            }

            double powerBefore = shooterPower;
            if (!trimTurret && dpadUp) {
                shooterPower += POWER_STEP_BIG;
            }
            if (!trimTurret && dpadDown) {
                shooterPower -= POWER_STEP_BIG;
            }
            if (!trimTurret && buttonA) {
                shooterPower += POWER_STEP_SMALL;
            }
            if (!trimTurret && buttonB) {
                shooterPower -= POWER_STEP_SMALL;
            }
            shooterPower = MathFunctions.clamp(shooterPower, 0, 1);
            if (shooterPower != powerBefore) {
                settleTimer.reset();
                double p = shooterPower;
                KLog.d(TAG, () -> "event=powerSet power=" + p);
            }
            if (kGamePad1.isButtonXFirstPressed()) {
                shooterOn = !shooterOn;
                settleTimer.reset();
                if (!shooterOn) {
                    shotArmed = false;
                }
                boolean on = shooterOn;
                KLog.d(TAG, () -> "event=shooterToggle on=" + on);
            }

            double hoodBefore = hoodPosition;
            if (!trimTurret && dpadRight) {
                hoodPosition += HOOD_STEP;
            }
            if (!trimTurret && dpadLeft) {
                hoodPosition -= HOOD_STEP;
            }
            if (trimTurret && buttonA) {
                hoodPosition += HOOD_STEP_FINE;
            }
            if (trimTurret && buttonB) {
                hoodPosition -= HOOD_STEP_FINE;
            }
            hoodPosition = MathFunctions.clamp(hoodPosition, ShooterInterpolationConfig.MIN_HOOD, ShooterInterpolationConfig.MAX_HOOD);
            shooter.getHood().setPosition(hoodPosition);
            if (hoodPosition != hoodBefore) {
                double h = hoodPosition;
                KLog.d(TAG, () -> "event=hoodSet hood=" + h);
            }

            boolean feeding = isPending(pushBall);
            if (wasFeeding && !feeding) {
                settleTimer.reset(); // the feed dropped the flywheel: let it recover before the next shot
                KLog.d(TAG, "event=feedDone");
            }
            wasFeeding = feeding;

            if (kGamePad1.isLeftBumperFirstPressed() && !feeding) {
                if (shotArmed) {
                    shotArmed = false;
                    message = "Shot cancelled";
                } else if (!shooterOn) {
                    message = "Shooter is off: press X first";
                } else {
                    shotArmed = true;
                    message = "";
                }
                boolean armed = shotArmed;
                KLog.d(TAG, () -> "event=shotArm armed=" + armed);
            }

            boolean aligned = turretAutoAlignTeleOp.isWithinRange();
            boolean settled = settleTimer.milliseconds() >= SETTLE_MS;
            if (shotArmed && aligned && settled) {
                shotArmed = false;
                Point aim = TurretAutoAlign.offsetTargetPoint(hiveTarget, SharedData.getOdometryWheelIMUPosition(), TurretConfig.TURRET_OFFSET_DEG);
                lastShot = String.format("%.2f,%.2f,%.2f,%.4f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f",
                        shooter.getRPS(), shooterPower, getBatteryVoltage(), hoodPosition, distanceMM,
                        hiveTarget.getX(), hiveTarget.getY(), TurretConfig.TURRET_OFFSET_DEG, aim.getX(), aim.getY());
                pushBall = new PushBall(stopper, intake);
                setLastShooterAction(pushBall); // updateActions() ticks it; it owns the intake + stopper until done
                feeding = true;
                String s = lastShot;
                KLog.d(TAG, () -> "event=shotRecorded row=" + s);
            }

            if (!feeding) {
                if (gamepad1.right_trigger > 0.1) {
                    intake.lower();
                    intake.run(1.0);
                } else {
                    intake.stop();
                }
            }

            shooter.setPower(shooterOn ? shooterPower : 0);

            if (kGamePad1.isButtonYFirstPressed()) {
                if (lastShot != null) {
                    fileWriter.writeLine(lastShot);
                    flush(fileWriter);
                    message = "SAVED: " + lastShot;
                    String s = lastShot;
                    KLog.d(TAG, () -> "event=saved row=" + s);
                } else {
                    message = "Nothing to save: shoot with LB first";
                    KLog.d(TAG, "event=saveRefused reason=noShot");
                }
            }

            for (String line : CONTROLS) {
                telemetry.addLine(line);
            }
            if (!message.isEmpty()) {
                telemetry.addLine(message);
            }
            telemetry.addData("Shot", feeding ? "FEEDING" : !shotArmed ? "idle"
                    : "armed, waiting for " + (aligned ? "" : "turret ") + (settled ? "" : "spin-up"));
            telemetry.addData("Shooter on / power", "%b / %.2f", shooterOn, shooterPower);
            telemetry.addData("Settled", "%.0f / %.0f ms", Math.min(settleTimer.milliseconds(), SETTLE_MS), SETTLE_MS);
            telemetry.addData("Balls", intake.getBallCount());
            telemetry.addData("Hood Position", "%.4f", hoodPosition);
            telemetry.addData("Current RPS", "%.2f", shooter.getRPS());
            telemetry.addData("Turret target / delta", "%.1f / %.1f deg",
                    Math.toDegrees(turretAutoAlignTeleOp.getTargetAngleRad()), turretAutoAlignTeleOp.getDeltaAngleDeg());
            telemetry.addData("Turret offset", "%.1f deg", TurretConfig.TURRET_OFFSET_DEG);
            telemetry.addData("Turret cmd / servo / clamped", "%.1f deg / %.4f / %b",
                    turret.getAngleDeg(), Turret.servoPosFromTurretDeg(turret.getAngleDeg()), turret.isClamped());
            telemetry.addData("Odometry pose", SharedData.getOdometryWheelIMUPosition());
            telemetry.addData("Target Cell", "%s (%.0f, %.0f)", hiveTarget == ShooterConfig.TARGET_POINT_1 ? "A (high y)" : "B (low y)",
                    hiveTarget.getX(), hiveTarget.getY());
            telemetry.addData("Turret to target (odometry)", "%.0f mm", distanceMM);
            telemetry.addData("Last shot (not saved until Y)", lastShot == null ? "none" : lastShot);
            telemetry.update();
        }

        TurretConfig.TURRET_OFFSET_DEG = 0; // static: don't leak the trim into TeleOp
        shooter.stop();
        intake.stop();
        driveTrain.setPower(0);
        fileWriter.close();
        KLog.d(TAG, "event=done file=RobotLogs/ pull: adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/RobotLogs/");
        cleanupRobot();
    }

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    private static void flush(KFileWriter writer) {
        try {
            writer.flush();
        } catch (java.io.IOException e) {
            KLog.e(TAG, "event=flushFailed", e);
        }
    }
}
