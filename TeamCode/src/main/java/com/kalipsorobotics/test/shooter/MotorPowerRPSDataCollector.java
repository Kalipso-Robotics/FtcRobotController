package com.kalipsorobotics.test.shooter;

import com.kalipsorobotics.modules.shooter.ShooterConfig;
import com.kalipsorobotics.utilities.KFileWriter;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KMotor;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;

/**
 * Motor Power and RPS Data Collection TeleOp
 *
 * This program tests shooter motors at different power levels (0.05 to 1.0)
 * and collects RPS data over time to analyze motor performance.
 *
 * Methodology:
 * 1. Loop through power levels from 0.05 to 1.0 in 0.05 increments
 * 2. For each power level:
 *    - Set power to both shooter1 and shooter2
 *    - Wait 2 seconds for motors to stabilize
 *    - Collect RPS data for 1 second (sampling every 50ms = 20 samples)
 *    - Record RPS_MAX, RPS_MIN, RPS_avg, and Battery_Voltage
 *
 * Output file columns: Power, RPS_MAX, RPS_MIN, RPS_avg, Battery_Voltage
 */
@TeleOp(name = "Motor Power RPS Data Collector", group = "Test")
public class MotorPowerRPSDataCollector extends LinearOpMode {

    // Test parameters
    private static final double POWER_START = 0.05;
    private static final double POWER_END = 1.0;
    private static final double POWER_INCREMENT = 0.05;
    private static final long STABILIZATION_TIME_MS = 4000; // 2 seconds
    private static final long DATA_COLLECTION_TIME_MS = 1000; // 1 second
    private static final long SAMPLE_INTERVAL_MS = 10; // Sample every 50ms
    private KMotor shooter1;
    private KMotor shooter2;
    private KFileWriter fileWriter;
    private OpModeUtilities opModeUtilities;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("=== Motor Power RPS Data Collector ===");
        telemetry.addLine("Initializing...");
        telemetry.update();

        // Initialize hardware
        initializeHardware();

        // Initialize file writer
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        fileWriter = new KFileWriter("MotorPowerRPSData", opModeUtilities);

        // Write CSV header
        fileWriter.writeLine("Power,RPS_MAX,RPS_MIN,RPS_avg,Battery_Voltage");

        telemetry.addLine("Initialization complete!");
        telemetry.addLine();
        telemetry.addLine("This test will:");
        telemetry.addLine("- Test power levels: 0.05 to 1.0 (step 0.05)");
        telemetry.addLine("- Wait 2s for stabilization");
        telemetry.addLine("- Collect RPS data for 1s");
        telemetry.addLine();
        telemetry.addLine("Press PLAY to start data collection");
        telemetry.update();

        KLog.d("MotorPowerRPSDataCollector", "Initialization complete. Waiting for start...");

        waitForStart();

        // Main data collection loop
        double currentPower = POWER_START;
        int testNumber = 1;
        int totalTests = (int) ((POWER_END - POWER_START) / POWER_INCREMENT) + 1;

        while (opModeIsActive() && currentPower <= POWER_END + 0.001) { // +0.001 for floating point precision
            // Get battery voltage
            double batteryVoltage = getBatteryVoltage();
            telemetry.addLine("=== Motor Power RPS Data Collection ===");
            telemetry.addData("Test", "%d / %d", testNumber, totalTests);
            telemetry.addData("Current Power", "%.2f", currentPower);
            telemetry.addLine();

            // Step 1: Set power to both motors
            shooter1.setPower(currentPower);
            shooter2.setPower(currentPower);

            telemetry.addLine("Status: Motors running at " + String.format("%.2f", currentPower) + " power");
            telemetry.addData("Stabilization Time", "%d ms", STABILIZATION_TIME_MS);
            telemetry.update();

            KLog.d("MotorPowerRPSDataCollector", String.format("Test %d/%d - Power: %.2f - Stabilizing...",
                testNumber, totalTests, currentPower));

            // Step 2: Wait for stabilization
            sleep(STABILIZATION_TIME_MS);

            telemetry.addLine("Status: Collecting RPS data...");
            telemetry.update();

            // Step 3: Collect RPS data for 1 second
            ArrayList<Double> rpsData1 = new ArrayList<>();
            ArrayList<Double> rpsData2 = new ArrayList<>();
            long startTime = System.currentTimeMillis();
            long endTime = startTime + DATA_COLLECTION_TIME_MS;

            while (System.currentTimeMillis() < endTime && opModeIsActive()) {
                double rps1 = shooter1.getRPS();
                double rps2 = shooter2.getRPS();
                rpsData1.add(rps1);
                rpsData2.add(rps2);

                telemetry.addLine("Status: Collecting RPS data...");
                telemetry.addData("Samples Collected", rpsData1.size());
                telemetry.addData("Shooter1 Current RPS", "%.2f", rps1);
                telemetry.addData("Shooter2 Current RPS", "%.2f", rps2);
                telemetry.update();

                sleep(SAMPLE_INTERVAL_MS);
            }

            // Step 4: Calculate statistics
            double rpsMax1 = calculateMax(rpsData1);
            double rpsMin1 = calculateMin(rpsData1);
            double rpsAvg1 = calculateAverage(rpsData1);

            double rpsMax2 = calculateMax(rpsData2);
            double rpsMin2 = calculateMin(rpsData2);
            double rpsAvg2 = calculateAverage(rpsData2);



            // Step 5: Write data to file (one line for each motor)
            String line1 = String.format("%.2f,%.2f,%.2f,%.2f,%.2f",
                currentPower, rpsMax1, rpsMin1, rpsAvg1, batteryVoltage);
            fileWriter.writeLine(line1);

//            // Also log shooter2 data (optional - comment out if you only want shooter1)
//            String line2 = String.format("%.2f,%.2f,%.2f,%.2f,%.2f",
//                currentPower, rpsMax2, rpsMin2, rpsAvg2, batteryVoltage);
//            // Uncomment the line below to also log shooter2 data
            // fileWriter.writeLine(line2);

            KLog.d("MotorPowerRPSDataCollector", String.format(
                "Test %d/%d Complete - Power: %.2f | Shooter1: MAX=%.2f MIN=%.2f AVG=%.2f | Battery: %.2fV",
                testNumber, totalTests, currentPower, rpsMax1, rpsMin1, rpsAvg1, batteryVoltage));

            telemetry.addLine("Status: Data recorded!");
            telemetry.addData("RPS Max (Shooter1)", "%.2f", rpsMax1);
            telemetry.addData("RPS Min (Shooter1)", "%.2f", rpsMin1);
            telemetry.addData("RPS Avg (Shooter1)", "%.2f", rpsAvg1);
            telemetry.addData("Battery Voltage", "%.2f V", batteryVoltage);
            telemetry.update();

            sleep(500); // Brief pause before next test

            // Move to next power level
            currentPower += POWER_INCREMENT;
            testNumber++;
        }

        // Stop motors
        shooter1.stop();
        shooter2.stop();

        // Close file
        fileWriter.close();

        telemetry.addLine("=== DATA COLLECTION COMPLETE ===");
        telemetry.addLine();
        telemetry.addLine("File saved to:");
        telemetry.addLine("/sdcard/Android/data/");
        telemetry.addLine("com.qualcomm.ftcrobotcontroller/");
        telemetry.addLine("files/OdometryLog/");
        telemetry.addLine();
        telemetry.addLine("Pull file using:");
        telemetry.addLine("adb pull /sdcard/Android/data/");
        telemetry.addLine("com.qualcomm.ftcrobotcontroller/");
        telemetry.addLine("files/OdometryLog/ ~/");
        telemetry.update();

        KLog.d("MotorPowerRPSDataCollector", "Data collection complete. File closed.");

        // Keep telemetry visible
        while (opModeIsActive()) {
            sleep(100);
        }
    }

    /**
     * Initialize shooter motors with proper configuration
     */
    private void initializeHardware() {
        // Initialize shooter1
        DcMotor motor1 = hardwareMap.dcMotor.get("shooter1");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1 = new KMotor(motor1,
            ShooterConfig.kp, ShooterConfig.ki, ShooterConfig.kd, ShooterConfig.kf);

        // Initialize shooter2
        DcMotor motor2 = hardwareMap.dcMotor.get("shooter2");
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter2 = new KMotor(motor2,
            ShooterConfig.kp, ShooterConfig.ki, ShooterConfig.kd, ShooterConfig.kf);

        KLog.d("MotorPowerRPSDataCollector", "Motors initialized successfully");
    }

    /**
     * Calculate maximum value from list
     */
    private double calculateMax(ArrayList<Double> data) {
        if (data.isEmpty()) return 0.0;
        double max = data.get(0);
        for (double value : data) {
            if (value > max) max = value;
        }
        return max;
    }

    /**
     * Calculate minimum value from list
     */
    private double calculateMin(ArrayList<Double> data) {
        if (data.isEmpty()) return 0.0;
        double min = data.get(0);
        for (double value : data) {
            if (value < min) min = value;
        }
        return min;
    }

    /**
     * Calculate average value from list
     */
    private double calculateAverage(ArrayList<Double> data) {
        if (data.isEmpty()) return 0.0;
        double sum = 0.0;
        for (double value : data) {
            sum += value;
        }
        return sum / data.size();
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
