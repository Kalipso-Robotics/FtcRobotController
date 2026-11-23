package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.FieldOrientedDriveAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
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
 * Output: RobotLogs/RPSPredictionData_*.csv with CurrentRPS,CurrentPower,CurrentVoltage,HoodPosition,DistanceToTargetMM.
 * Distance is from odometry to the Hive Cell on the robot's end (SharedData.getTargetPoint).
 * Pattern source: this file's earlier LinearOpMode version, ported to KOpMode + startOdometry.
 *
 * Turret aims at the Hive target the whole time (TurretAutoAlign, ticked by updateActions()).
 *
 * Controls (GP1):
 *   sticks            field-centric drive (IMU yaw; "forward" = the way the robot faced at init)
 *   dpad up / down    shooter power +/- POWER_STEP_BIG        A / B    power +/- POWER_STEP_SMALL
 *   dpad right / left hood +/- HOOD_STEP (clamped MIN_HOOD..MAX_HOOD)
 *   X (hold)          run the shooter at the set power
 *   RT (hold)         intake + stopper closed (load)
 *   LT (hold)         stopper open + intake (shoot); the press records RPS/power/voltage/hood/distance
 *   Y                 save the last recorded shot to the file
 *   RB                rezero: robot pushed into the loading-zone corner facing +x -> FieldConfig.CORNER_ZERO_POSE
 *
 * Turret debug: KLog tag Test_Shooter, "event=turret" every TURRET_LOG_MS: pose, target, field aim, turret deg,
 * servo position. Positive turretDeg must swing the turret to the robot's RIGHT (clockwise from above).
 */
@TeleOp(name = "RPS Prediction Data Collector", group = "Test")
public class RPSPredictionDataCollector extends KOpMode {
    public static final String TAG = "Test_Shooter";
    private static final double POWER_STEP_BIG = 0.05;
    private static final double POWER_STEP_SMALL = 0.01;
    private static final double HOOD_STEP = 0.01;
    private static final long TURRET_LOG_MS = 250;

    private static final String[] CONTROLS = {
            "sticks: field-centric drive   turret auto-aims at the Hive",
            "dpad up/down: power +/-" + POWER_STEP_BIG + "   A/B: power +/-" + POWER_STEP_SMALL,
            "dpad right/left: hood +/-" + HOOD_STEP,
            "X hold: run shooter   RT hold: load (intake, stopper closed)",
            "LT hold: shoot (records the shot)   Y: save last shot",
            "RB: rezero odometry (robot in corner, facing +x)",
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

        turretAutoAlignTeleOp = new TurretAutoAlign(opModeUtilities, turret); // follows SharedData.getTargetPoint(); updateActions() ticks it
        turretAutoAlignTeleOp.setTurretRunMode(TurretRunMode.RUN_USING_ODOMETRY);
        KLog.d(TAG, () -> String.format("event=turretConfig degPerServoDeg=%.4f servoRangeDeg=%.1f zeroPos=%.3f dir=%d limitsDeg=%.1f..%.1f pivot=%s startPose=%s",
                TurretConfig.TURRET_DEG_PER_SERVO_DEG, TurretConfig.SERVO_RANGE_DEG, TurretConfig.SERVO_ZERO_POS,
                TurretConfig.SERVO_DIRECTION, TurretConfig.MIN_TURRET_DEG, TurretConfig.MAX_TURRET_DEG, TurretConfig.ROBOT_REL_TURRET_POINT,
                SharedData.getOdometryWheelIMUPosition()));
        long lastTurretLogMs = 0;

        KFileWriter fileWriter = new KFileWriter("RPSPredictionData", opModeUtilities);
        fileWriter.writeLine("CurrentRPS,CurrentPower,CurrentVoltage,HoodPosition,DistanceToTargetMM");
        flush(fileWriter);

        double shooterPower = 0;
        double hoodPosition = ShooterInterpolationConfig.MIN_HOOD;
        shooter.getHood().setPosition(hoodPosition);
        Point hiveTarget = null;
        String lastShot = null;
        String message = "";
        boolean wasShooting = false;

        for (String line : CONTROLS) {
            telemetry.addLine(line);
        }
        telemetry.addLine("Press PLAY to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            hiveTarget = SharedData.getTargetPoint();
            double distanceMM = SharedData.getOdometryWheelIMUPosition().toPoint().distanceTo(hiveTarget);
            if (kGamePad1.isRightBumperFirstPressed()) {
                setLastZeroAction(new ResetOdometryToPos(new Position(FieldConfig.CORNER_ZERO_POSE))); // updateActions() applies it
                message = "Rezeroed to corner " + FieldConfig.CORNER_ZERO_POSE;
                KLog.d(TAG, () -> "event=rezero from=" + SharedData.getOdometryWheelIMUPosition() + " to=" + FieldConfig.CORNER_ZERO_POSE);
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
            if (kGamePad1.isDpadUpFirstPressed()) {
                shooterPower += POWER_STEP_BIG;
            }
            if (kGamePad1.isDpadDownFirstPressed()) {
                shooterPower -= POWER_STEP_BIG;
            }
            if (kGamePad1.isButtonAFirstPressed()) {
                shooterPower += POWER_STEP_SMALL;
            }
            if (kGamePad1.isButtonBFirstPressed()) {
                shooterPower -= POWER_STEP_SMALL;
            }
            shooterPower = MathFunctions.clamp(shooterPower, 0, 1);
            if (shooterPower != powerBefore) {
                double p = shooterPower;
                KLog.d(TAG, () -> "event=powerSet power=" + p);
            }

            double hoodBefore = hoodPosition;
            if (kGamePad1.isDpadRightFirstPressed()) {
                hoodPosition += HOOD_STEP;
            }
            if (kGamePad1.isDpadLeftFirstPressed()) {
                hoodPosition -= HOOD_STEP;
            }
            hoodPosition = MathFunctions.clamp(hoodPosition, ShooterInterpolationConfig.MIN_HOOD, ShooterInterpolationConfig.MAX_HOOD);
            shooter.getHood().setPosition(hoodPosition);
            if (hoodPosition != hoodBefore) {
                double h = hoodPosition;
                KLog.d(TAG, () -> "event=hoodSet hood=" + h);
            }

            boolean shooting = gamepad1.left_trigger > 0.1;
            if (gamepad1.right_trigger > 0.1) {
                intake.run(1.0);
                stopper.setPosition(ModuleConfig.STOPPER_SERVO_CLOSED_POS);
            } else if (shooting) {
                stopper.setPosition(ModuleConfig.STOPPER_SERVO_OPEN_POS);
                intake.run(1.0);
                if (!wasShooting) {
                    lastShot = String.format("%.2f,%.2f,%.2f,%.4f,%.2f",
                            shooter.getRPS(), shooterPower, getBatteryVoltage(), hoodPosition, distanceMM);
                    String s = lastShot;
                    KLog.d(TAG, () -> "event=shotRecorded row=" + s);
                }
            } else {
                intake.stop();
            }
            wasShooting = shooting;

            shooter.setPower(gamepad1.x ? shooterPower : 0);

            if (kGamePad1.isButtonYFirstPressed()) {
                if (lastShot != null) {
                    fileWriter.writeLine(lastShot);
                    flush(fileWriter);
                    message = "SAVED: " + lastShot;
                    String s = lastShot;
                    KLog.d(TAG, () -> "event=saved row=" + s);
                } else {
                    message = "Nothing to save: shoot with LT first";
                    KLog.d(TAG, "event=saveRefused reason=noShot");
                }
            }

            for (String line : CONTROLS) {
                telemetry.addLine(line);
            }
            if (!message.isEmpty()) {
                telemetry.addLine(message);
            }
            telemetry.addData("Shooter Power", "%.2f", shooterPower);
            telemetry.addData("Hood Position", "%.4f", hoodPosition);
            telemetry.addData("Current RPS", "%.2f", shooter.getRPS());
            telemetry.addData("Turret target / delta", "%.1f / %.1f deg",
                    Math.toDegrees(turretAutoAlignTeleOp.getTargetAngleRad()), turretAutoAlignTeleOp.getDeltaAngleDeg());
            telemetry.addData("Turret cmd / servo / clamped", "%.1f deg / %.4f / %b",
                    turret.getAngleDeg(), Turret.servoPosFromTurretDeg(turret.getAngleDeg()), turret.isClamped());
            telemetry.addData("Odometry pose", SharedData.getOdometryWheelIMUPosition());
            telemetry.addData("Odometry distance to target", "%.0f mm", distanceMM);
            telemetry.addData("Last shot (not saved until Y)", lastShot == null ? "none" : lastShot);
            telemetry.update();
        }

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
