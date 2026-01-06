package com.kalipsorobotics.test.shooter;

import com.kalipsorobotics.actions.cameraVision.AprilTagDetectionAction;
import com.kalipsorobotics.actions.turret.TurretAutoAlignTeleOp;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.test.turret.TurretRunMode;
import com.kalipsorobotics.utilities.KFileWriter;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * RPS Prediction Data Collection TeleOp
 *
 * This program collects data for shooter RPS prediction by allowing manual
 * power control and selective data recording.
 *
 * Workflow:
 * 1. Adjust shooter power using DPad/A/B buttons
 * 2. Press X to run the shooter at the set power
 * 3. Press Left Trigger to record shoot data (RPS, Power, Distance)
 * 4. Press Y to save the last recorded shoot to file
 *
 * Controls:
 * - DPad Up: Increase shooter power by 0.05
 * - DPad Down: Decrease shooter power by 0.05
 * - DPad Left: Decrease hood position by 0.05
 * - DPad Right: Increase hood position by 0.05
 * - A Button: Increase shooter power by 0.01
 * - B Button: Decrease shooter power by 0.01
 * - X Button: Run shooter at current power
 * - Right Trigger: Run intake full speed + Close stopper
 * - Left Trigger: Record shoot data (RPS, Power, Distance to target)
 * - Y Button: Save last recorded shoot to file
 *
 * Data recorded per shoot:
 * - Current RPS
 * - Current Power
 * - Current Voltage
 * - Hood Position
 * - Distance to target (MM)
 */
@TeleOp(name = "RPS Prediction Data Collector", group = "Test")
public class RPSPredictionDataCollector extends LinearOpMode {

    /**
     * Inner class to hold shoot data
     */
    private static class ShootData {
        double currentRPS;
        double currentPower;
        double currentVoltage;
        double hoodPosition;
        double distanceToTargetMM;

        ShootData(double rps, double power, double voltage, double hood, double distance) {
            this.currentRPS = rps;
            this.currentPower = power;
            this.currentVoltage = voltage;
            this.hoodPosition = hood;
            this.distanceToTargetMM = distance;
        }

        @Override
        public String toString() {
            return String.format("RPS=%.2f, Power=%.2f, Voltage=%.2fV, Hood=%.2f, Distance=%.2fmm",
                currentRPS, currentPower, currentVoltage, hoodPosition, distanceToTargetMM);
        }

        String toCSV() {
            return String.format("%.2f,%.2f,%.2f,%.2f,%.2f", currentRPS, currentPower, currentVoltage, hoodPosition, distanceToTargetMM);
        }
    }

    private Shooter shooter;
    private Intake intake;
    private Stopper stopper;
    private Odometry odometry;
    private DriveTrain driveTrain;
    private IMUModule imuModule;
    private TurretAutoAlignTeleOp turretAutoAlignTeleop;
    private Turret turret;
    private KFileWriter fileWriter;
    private OpModeUtilities opModeUtilities;

    private double shooterPower = 0.0;
    private double hoodPosition = 0.8;
    private ShootData lastShoot = null;
    private boolean wasLeftTriggerPressed = false;

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
        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);
        AprilTagDetectionAction aprilTagDetectionAction = new AprilTagDetectionAction(opModeUtilities, turret, 24, AllianceColor.RED);
        turretAutoAlignTeleop = new TurretAutoAlignTeleOp(opModeUtilities, turret, AllianceColor.RED);
        turretAutoAlignTeleop.setTurretRunMode(TurretRunMode.RUN_USING_ODOMETRY);
        // Write CSV header
        fileWriter.writeLine("CurrentRPS,CurrentPower,CurrentVoltage,HoodPosition,DistanceToTargetMM");

        telemetry.addLine("Initialization complete!");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("- DPad Up/Down: ±0.05 power");
        telemetry.addLine("- DPad Left/Right: ±0.05 hood");
        telemetry.addLine("- A Button: +0.01 power");
        telemetry.addLine("- B Button: -0.01 power");
        telemetry.addLine("- X Button: Run shooter");
        telemetry.addLine("- Right Trigger: Intake + Close stopper");
        telemetry.addLine("- Left Trigger: Record shoot data");
        telemetry.addLine("- Y Button: Save last shoot to file");
        telemetry.addLine();
        telemetry.addLine("Press PLAY to start");
        telemetry.update();

        KLog.d("RPSPredictionDataCollector", "Initialization complete. Waiting for start...");
        shooter.getHood().setPosition(0.8);
        waitForStart();

        // Main control loop
        while (opModeIsActive()) {
            // Update odometry
            odometry.updateAll();
            turretAutoAlignTeleop.updateCheckDone();
            aprilTagDetectionAction.updateCheckDone();

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

            // ========== Hood Control ==========
            if (gamepad1.dpad_right) {
                hoodPosition += 0.01;
            }
            if (gamepad1.dpad_left) {
                hoodPosition -= 0.01;
            }

            // Clamp hood position between 0 and 1
            if (hoodPosition < 0) {
                hoodPosition = 0;
            }
            if (hoodPosition > 1.0) {
                hoodPosition = 1.0;
            }

            // Apply hood position
            shooter.getHood().setPosition(hoodPosition);

            // ========== Trigger Controls ==========
            if (gamepad1.right_trigger > 0.1) {
                // Right trigger: Run intake + Close stopper
                intake.getIntakeMotor().setPower(1.0);
                stopper.getStopper().setPosition(stopper.STOPPER_SERVO_CLOSED_POS);
            } else if (gamepad1.left_trigger > 0.1) {
                // Left trigger: Record shoot data
                stopper.getStopper().setPosition(stopper.STOPPER_SERVO_OPEN_POS);
                intake.getIntakeMotor().setPower(1.0);

                // Record shoot data when trigger is first pressed (edge detection)
                if (!wasLeftTriggerPressed) {
                    double currentRPS = shooter.getRPS();
                    double currentVoltage = getBatteryVoltage();
                    double distanceToTargetMM = calculateDistanceToTarget();

                    lastShoot = new ShootData(currentRPS, shooterPower, currentVoltage, hoodPosition, distanceToTargetMM);

                    // Log to KLog
                    KLog.d("RPSPredictionData", "SHOOT RECORDED: " + lastShoot.toString());

                    wasLeftTriggerPressed = true;
                }
            } else {
                // No triggers: Stop intake
                intake.getIntakeMotor().setPower(0);
                wasLeftTriggerPressed = false;
            }

            if (gamepad1.x) {
                shooter.setPower(shooterPower);
            } else {
                shooter.setPower(0);
            }

            // ========== Y Button: Save Last Shoot to File ==========
            if (gamepad1.y) {
                if (lastShoot != null) {
                    fileWriter.writeLine(lastShoot.toCSV());
                    KLog.d("RPSPredictionData", "SAVED TO FILE: " + lastShoot.toString());
                    telemetry.addLine("*** SAVED TO FILE ***");
                } else {
                    KLog.d("RPSPredictionData", "No shoot data to save!");
                    telemetry.addLine("*** NO DATA TO SAVE ***");
                }
            }

            // ========== Telemetry ==========
            telemetry.addLine("=== RPS Prediction Data Collector ===");
            telemetry.addLine();
            telemetry.addData("Shooter Power", "%.2f", shooterPower);
            telemetry.addData("Hood Position", "%.2f", hoodPosition);
            telemetry.addData("Current RPS", "%.2f", shooter.getRPS());
            telemetry.addData("Limelight Distance to Target", "%.2f mm", calculateDistanceToTarget());
            telemetry.addLine();

            if (lastShoot != null) {
                telemetry.addLine("--- Last Recorded Shoot ---");
                telemetry.addData("Last RPS", "%.2f", lastShoot.currentRPS);
                telemetry.addData("Last Power", "%.2f", lastShoot.currentPower);
                telemetry.addData("Last Voltage", "%.2f V", lastShoot.currentVoltage);
                telemetry.addData("Last Hood", "%.2f", lastShoot.hoodPosition);
                telemetry.addData("Last Distance", "%.2f mm", lastShoot.distanceToTargetMM);
            } else {
                telemetry.addLine("--- No Shoot Recorded Yet ---");
            }

            telemetry.addLine();
            telemetry.addLine("Controls:");
            telemetry.addLine("DPad Up/Down: ±0.05 power");
            telemetry.addLine("DPad Left/Right: ±0.05 hood");
            telemetry.addLine("A/B: ±0.01 power");
            telemetry.addLine("X: Run shooter");
            telemetry.addLine("Right Trigger: Intake + Close");
            telemetry.addLine("Left Trigger: Record shoot");
            telemetry.addLine("Y: Save to file");
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
        return SharedData.getLimelightRawPosition().getApriLTagDistanceToCamMM();

//        Position currentPosition = odometry.update();
//
//        double currentX = currentPosition.getX();
//        double currentY = currentPosition.getY();
//
//        double targetX = TurretConfig.X_INIT_SETUP_MM;
//        double targetY = TurretConfig.Y_INIT_SETUP_MM;
//
//        double dx = targetX - currentX;
//        double dy = targetY - currentY;
//
//        double distance = Math.sqrt((dx * dx) + (dy * dy));
//
//        return distance;
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
