package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * Open-loop check: what RPS a given motor power gives, at the battery voltage, hood and distance at the time.
 * Output: RobotLogs/RPSPredictionData_*.csv with CurrentRPS,CurrentPower,CurrentVoltage,HoodPosition,DistanceToTargetMM.
 * Distance is from odometry to the Hive Cell on the robot's end (FieldConfig.hiveTargetForSide, refreshed every loop).
 * Pattern source: this file's earlier LinearOpMode version, ported to KOpMode + startOdometry.
 *
 * Controls (GP1):
 *   dpad up / down    shooter power +/- POWER_STEP_BIG        A / B    power +/- POWER_STEP_SMALL
 *   dpad right / left hood +/- HOOD_STEP (clamped MIN_HOOD..MAX_HOOD)
 *   X (hold)          run the shooter at the set power
 *   RT (hold)         intake + stopper closed (load)
 *   LT (hold)         stopper open + intake (shoot); the press records RPS/power/voltage/hood/distance
 *   Y                 save the last recorded shot to the file
 */
@TeleOp(name = "RPS Prediction Data Collector", group = "Test")
public class RPSPredictionDataCollector extends KOpMode {
    public static final String TAG = "Test_Shooter";
    private static final double POWER_STEP_BIG = 0.05;
    private static final double POWER_STEP_SMALL = 0.01;
    private static final double HOOD_STEP = 0.01;

    private static final String[] CONTROLS = {
            "dpad up/down: power +/-" + POWER_STEP_BIG + "   A/B: power +/-" + POWER_STEP_SMALL,
            "dpad right/left: hood +/-" + HOOD_STEP,
            "X hold: run shooter   RT hold: load (intake, stopper closed)",
            "LT hold: shoot (records the shot)   Y: save last shot",
    };

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        Shooter shooter = new Shooter(opModeUtilities);
        Intake intake = new Intake(opModeUtilities);
        Stopper stopper = new Stopper(opModeUtilities);
        OpModeUtilities.startOdometry(opModeUtilities, odoExecutorService, driveTrain, imuModule, null); // keeps the pose from auto, else (0,0,0)

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
            hiveTarget = FieldConfig.hiveTargetForSide(SharedData.getOdometryWheelIMUPosition().getY(), hiveTarget);
            double distanceMM = SharedData.getOdometryWheelIMUPosition().toPoint().distanceTo(hiveTarget);

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
            telemetry.addData("Odometry distance to target", "%.0f mm", distanceMM);
            telemetry.addData("Last shot (not saved until Y)", lastShot == null ? "none" : lastShot);
            telemetry.update();
        }

        shooter.stop();
        intake.stop();
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
