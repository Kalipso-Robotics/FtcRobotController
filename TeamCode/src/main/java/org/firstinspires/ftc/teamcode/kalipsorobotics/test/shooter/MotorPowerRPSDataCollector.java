package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KMotor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;

/**
 * Motor Power and RPS Data Collection TeleOp
 *
 * This program tests the shooter motor at different power levels
 * and collects RPS data over time to analyze motor performance.
 * Pattern source: this file's pre-milestone-2 two-motor version, reduced to one motor.
 *
 * To measure ShooterConfig.kf: read the kf column (kf = effective power / steady RPS, where effective power = power * battery voltage / DEFAULT_VOLTAGE
 * when ShooterConfig.USE_VOLTAGE_COMPENSATION, matching KMotor.goToRPS) at about 0.8 power
 * with a full battery, then paste it into ShooterConfig.kf. kf is a measured value, not Dashboard-tuned.
 *
 * Methodology:
 * 1. Loop through power levels from 0.05 to 1.0 in 0.05 increments
 * 2. For each power level:
 *    - Set power to the shooter motor
 *    - Wait 2 seconds for motors to stabilize
 *    - Collect RPS data for 1 second (sampling every 50ms = 20 samples)
 *    - Record RPS_MAX, RPS_MIN, RPS_avg, and Battery_Voltage
 *
 * Output file columns: Power, RPS_MAX, RPS_MIN, RPS_avg, Battery_Voltage, kf
 */

@TeleOp(name = "MotorPowerRPSDataCollector", group = "Test")
public class MotorPowerRPSDataCollector extends LinearOpMode {

    // Test parameters
    private static final double POWER_START = 0.3;
    private static final double POWER_END = 1;
    private static final double POWER_INCREMENT = 0.1;
    private static final long STABILIZATION_TIME_MS = 6000; // 2 seconds
    private static final long DATA_COLLECTION_TIME_MS = 1000; // 1 second
    private static final long SAMPLE_INTERVAL_MS = 10; // Sample every 50ms/
    private static final String TAG = "Test_Shooter";
    private KMotor shooter;
    private KFileWriter fileWriter;
    private OpModeUtilities opModeUtilities;

    /** kf = effective power / rps; effective power mirrors KMotor.goToRPS voltage compensation. */
    static double computeKf(double power, double batteryVoltage, double rpsAvg) {
        if (rpsAvg <= 0) {
            return 0;
        }
        double effectivePower = ShooterConfig.USE_VOLTAGE_COMPENSATION
                ? power * batteryVoltage / ShooterInterpolationConfig.DEFAULT_VOLTAGE : power;
        return effectivePower / rpsAvg;
    }

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
        fileWriter.writeLine("Power,RPS_MAX,RPS_MIN,RPS_avg,Battery_Voltage,kf");

        telemetry.addLine("Initialization complete!");
        telemetry.addLine();
        telemetry.addLine("This test will:");
        telemetry.addLine("- Test power levels: 0.05 to 1.0 (step 0.05)");
        telemetry.addLine("- Wait 2s for stabilization");
        telemetry.addLine("- Collect RPS data for 1s");
        telemetry.addLine();
        telemetry.addLine("Press PLAY to start data collection");
        telemetry.update();

        KLog.d(TAG, "event=initDone");

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

            // Step 1: Set power
            shooter.setPower(currentPower);

            telemetry.addLine("Status: Motors running at " + String.format("%.2f", currentPower) + " power");
            telemetry.addData("Stabilization Time", "%d ms", STABILIZATION_TIME_MS);
            telemetry.update();

            double finalCurrentPower = currentPower;
            int finalTestNumber = testNumber;
            KLog.d(TAG, () -> String.format("event=stabilizing test=%d/%d power=%.2f",
                    finalTestNumber, totalTests, finalCurrentPower));

            // Step 2: Wait for stabilization
            sleep(STABILIZATION_TIME_MS);

            telemetry.addLine("Status: Collecting RPS data...");
            telemetry.update();

            // Step 3: Collect RPS data for 1 second
            ArrayList<Double> rpsData = new ArrayList<>();
            long startTime = System.currentTimeMillis();
            long endTime = startTime + DATA_COLLECTION_TIME_MS;

            while (System.currentTimeMillis() < endTime && opModeIsActive()) {
                double rps = shooter.getRPS();
                rpsData.add(rps);

                telemetry.addLine("Status: Collecting RPS data...");
                telemetry.addData("Samples Collected", rpsData.size());
                telemetry.addData("Shooter Current RPS", "%.2f", rps);
                telemetry.update();

                sleep(SAMPLE_INTERVAL_MS);
            }

            // Step 4: Calculate statistics
            double rpsMax = calculateMax(rpsData);
            double rpsMin = calculateMin(rpsData);
            double rpsAvg = calculateAverage(rpsData);
            double kf = computeKf(currentPower, batteryVoltage, rpsAvg);

            fileWriter.writeLine(String.format("%.2f,%.2f,%.2f,%.2f,%.2f,%.5f",
                currentPower, rpsMax, rpsMin, rpsAvg, batteryVoltage, kf));

            int finalTestNumber1 = testNumber;
            double finalCurrentPower1 = currentPower;
            KLog.d(TAG, () -> String.format(
                "event=testDone test=%d/%d power=%.2f rpsMax=%.2f rpsMin=%.2f rpsAvg=%.2f kf=%.5f batteryV=%.2f",
                    finalTestNumber1, totalTests, finalCurrentPower1, rpsMax, rpsMin, rpsAvg, kf, batteryVoltage));

            telemetry.addLine("Status: Data recorded!");
            telemetry.addData("RPS Max", "%.2f", rpsMax);
            telemetry.addData("RPS Min", "%.2f", rpsMin);
            telemetry.addData("RPS Avg", "%.2f", rpsAvg);
            telemetry.addData("kf (effective power / RPS)", "%.5f", kf);
            telemetry.addData("Battery Voltage", "%.2f V", batteryVoltage);
            telemetry.update();

            sleep(500); // Brief pause before next test

            // Move to next power level
            currentPower += POWER_INCREMENT;
            testNumber++;
        }

        // Stop motors
        shooter.stopAndResetPID();

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
        telemetry.addLine("files/OdometryLog/ ~/Downloads");
        telemetry.update();

        KLog.d(TAG, "event=done file=closed");

        // Keep telemetry visible
        while (opModeIsActive()) {
            sleep(100);
        }
    }

    /**
     * Initialize the shooter motor with proper configuration
     */
    private void initializeHardware() {
        DcMotor motor = hardwareMap.dcMotor.get("shooter");
        motor.setDirection(DcMotorSimple.Direction.REVERSE); // same as Shooter
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter = new KMotor(motor,
            ShooterConfig.kp, ShooterConfig.ki, ShooterConfig.kd, ShooterConfig.kf);

        KLog.d(TAG, "event=motorsInit");
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
