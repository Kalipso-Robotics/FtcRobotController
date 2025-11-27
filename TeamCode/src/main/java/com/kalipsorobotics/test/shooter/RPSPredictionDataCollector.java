package com.kalipsorobotics.test.shooter;

import com.kalipsorobotics.actions.turret.TurretConfig;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.KFileWriter;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * RPS Prediction Data Collection TeleOp
 *
 * This program collects data for shooter RPS prediction by allowing manual
 * power control and logging shooter performance metrics along with distance to target.
 *
 * Controls:
 * - DPad Up: Increase shooter power by 0.05
 * - DPad Down: Decrease shooter power by 0.05
 * - A Button: Increase shooter power by 0.01
 * - B Button: Decrease shooter power by 0.01
 * - Right Trigger: Run intake full speed + Close stopper
 * - Left Trigger: Open stopper + Run intake + Run shooter at current power
 * - Else: Stop shooter
 *
 * Data logged:
 * - Current RPS
 * - Current Power
 * - Current Voltage
 * - Distance to target (MM)
 */
@TeleOp(name = "RPS Prediction Data Collector", group = "Test")
public class RPSPredictionDataCollector extends LinearOpMode {

    private Shooter shooter;
    private Intake intake;
    private Stopper stopper;
    private Odometry odometry;
    private DriveTrain driveTrain;
    private IMUModule imuModule;

    private KFileWriter fileWriter;
    private OpModeUtilities opModeUtilities;

    private double shooterPower = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("=== RPS Prediction Data Collector ===");
        telemetry.addLine("Initializing...");
        telemetry.update();

        // Initialize hardware
        initializeHardware();

        // Initialize file writer
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        fileWriter = new KFileWriter("RPSPredictionData", opModeUtilities);

        // Write CSV header
        fileWriter.writeLine("CurrentRPS,CurrentPower,CurrentVoltage,DistanceToTargetMM");

        telemetry.addLine("Initialization complete!");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("- DPad Up: +0.05 power");
        telemetry.addLine("- DPad Down: -0.05 power");
        telemetry.addLine("- A Button: +0.01 power");
        telemetry.addLine("- B Button: -0.01 power");
        telemetry.addLine("- Right Trigger: Intake + Close stopper");
        telemetry.addLine("- Left Trigger: Open stopper + Intake + Shoot");
        telemetry.addLine();
        telemetry.addLine("Press PLAY to start");
        telemetry.update();

        KLog.d("RPSPredictionDataCollector", "Initialization complete. Waiting for start...");

        waitForStart();

        // Main control loop
        while (opModeIsActive()) {
            // Update odometry
            odometry.updateAll();

            // ========== Power Control ==========
            if (gamepad1.dpad_up) {
                shooterPower += 0.05;
            }
            if (gamepad1.dpad_down) {
                shooterPower -= 0.05;
            }
            if (gamepad1.a) {
                shooterPower += 0.01;
            }
            if (gamepad1.b) {
                shooterPower -= 0.01;
            }

            // Clamp shooter power between 0 and 1
            if (shooterPower < 0) {
                shooterPower = 0;
            }
            if (shooterPower > 1.0) {
                shooterPower = 1.0;
            }

            // ========== Trigger Controls ==========
            if (gamepad1.right_trigger > 0.1) {
                // Right trigger: Run intake + Close stopper
                intake.getIntakeMotor().setPower(1.0);
                stopper.getStopper().setPosition(Stopper.STOPPER_SERVO_CLOSED_POS);
            } else if (gamepad1.left_trigger > 0.1) {
                // Left trigger: Open stopper + Run intake + Run shooter
                stopper.getStopper().setPosition(Stopper.STOPPER_SERVO_OPEN_POS);
                intake.getIntakeMotor().setPower(1.0);
                shooter.setPower(shooterPower);
            } else {
                // No triggers: Stop shooter and intake
                shooter.stop();
                intake.getIntakeMotor().setPower(0);
            }

            // ========== Data Collection ==========
            double currentRPS = shooter.getRPS();
            double currentVoltage = getBatteryVoltage();
            double distanceToTargetMM = calculateDistanceToTarget();

            // Log data to file
            String dataLine = String.format("%.2f,%.2f,%.2f,%.2f",
                currentRPS, shooterPower, currentVoltage, distanceToTargetMM);
            fileWriter.writeLine(dataLine);

            // Also log to KLog
            KLog.d("RPSPredictionData", String.format(
                "RPS=%.2f, Power=%.2f, Voltage=%.2fV, Distance=%.2fmm",
                currentRPS, shooterPower, currentVoltage, distanceToTargetMM));

            // ========== Telemetry ==========
            telemetry.addLine("=== RPS Prediction Data Collector ===");
            telemetry.addLine();
            telemetry.addData("Shooter Power", "%.2f", shooterPower);
            telemetry.addData("Current RPS", "%.2f", currentRPS);
            telemetry.addData("Battery Voltage", "%.2f V", currentVoltage);
            telemetry.addData("Distance to Target", "%.2f mm", distanceToTargetMM);
            telemetry.addLine();
            telemetry.addLine("Controls:");
            telemetry.addLine("DPad Up/Down: ±0.05 power");
            telemetry.addLine("A/B: ±0.01 power");
            telemetry.addLine("Right Trigger: Intake + Close");
            telemetry.addLine("Left Trigger: Shoot");
            telemetry.update();
        }

        // Cleanup
        shooter.stop();
        intake.getIntakeMotor().setPower(0);
        fileWriter.close();

        telemetry.addLine("=== DATA COLLECTION COMPLETE ===");
        telemetry.addLine();
        telemetry.addLine("File saved to:");
        telemetry.addLine("/sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/OdometryLog/");
        telemetry.update();

        KLog.d("RPSPredictionDataCollector", "Data collection complete. File closed.");
        KLog.d("RPSPredictionDataCollector", "Pull files using: adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/OdometryLog/ ~/");

        // Keep telemetry visible
        while (opModeIsActive()) {
            sleep(100);
        }
    }

    /**
     * Initialize all hardware components
     */
    private void initializeHardware() {
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        // Initialize drive train and IMU for odometry
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        imuModule = IMUModule.getInstance(opModeUtilities);

        // Initialize odometry
        odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);

        // Initialize shooter, intake, and stopper
        shooter = new Shooter(opModeUtilities);
        intake = new Intake(opModeUtilities);
        stopper = new Stopper(opModeUtilities);

        KLog.d("RPSPredictionDataCollector", "Hardware initialized successfully");
    }

    /**
     * Calculate distance to target based on current odometry position
     * and target position from TurretConfig
     */
    private double calculateDistanceToTarget() {
        Position currentPosition = odometry.update();

        double currentX = currentPosition.getX();
        double currentY = currentPosition.getY();

        double targetX = TurretConfig.X_INIT_SETUP_MM;
        double targetY = TurretConfig.Y_INIT_SETUP_MM;

        double dx = targetX - currentX;
        double dy = targetY - currentY;

        double distance = Math.sqrt((dx * dx) + (dy * dy));

        return distance;
    }

    /**
     * Get battery voltage from voltage sensor
     */
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
}
