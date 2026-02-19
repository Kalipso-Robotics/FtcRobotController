package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;

//@TeleOp(name = "DriveTrainKsTuner", group = "Test")
public class DriveTrainKsTuner extends KOpMode {
    private DriveTrain driveTrain;
    private IMU imu;
    private DcMotor rightEncoder;
    private DcMotor leftEncoder;

    private KFileWriter fileWriter;
    private static final String LOG_TAG = "DriveTrainKsTuner";

    // Test parameters
    private static final double POWER_INCREMENT = 0.005;  // Small increments for precision
    private static final double MAX_POWER = 0.5;
    private static final double MIN_POWER = 0.0;

    // Velocity thresholds
    private static final double LINEAR_VELOCITY_THRESHOLD = 5.0;  // ticks/second
    private static final double ANGULAR_VELOCITY_THRESHOLD = 1.0; // degrees/second

    // Test timing
    private static final double TEST_DURATION = 1.0;  // seconds to test each power level
    private static final double SETTLE_TIME = 0.5;    // seconds to wait before measurement
    private static final int TRIALS_PER_POWER = 3;    // number of trials to average

    // Results storage
    private final List<Double> linearValidPowers = new ArrayList<>();
    private final List<Double> rotationalValidPowers = new ArrayList<>();

    private double minLinearPower = -1;
    private double minRotationalPower = -1;

    @Override
    public void runOpMode() {
        // Initialize hardware
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        imu = imuModule.getIMU();

        rightEncoder = driveTrain.getRightEncoder();
        leftEncoder = driveTrain.getLeftEncoder();

        // Initialize file writer
        try {
            fileWriter = new KFileWriter("DriveTrainKsTuner", opModeUtilities);
            KLog.d(LOG_TAG, "File writer initialized successfully");
        } catch (RuntimeException e) {
            KLog.e(LOG_TAG, "Failed to initialize file writer", e);
            fileWriter = null;
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Log File", fileWriter != null ? "Ready" : "FAILED TO CREATE");
        telemetry.addData("Press Start", "to begin tuning");
        telemetry.update();
        KLog.d(LOG_TAG, "Initialization complete, waiting for start");

        waitForStart();

        // Log start
        KLog.d(LOG_TAG, "=== DriveTrainKsTuner Started ===");
        writeLog("=== DriveTrainKsTuner Started ===");
        writeLog("Power Increment: " + POWER_INCREMENT);
        writeLog("Max Power: " + MAX_POWER);
        writeLog("Linear Velocity Threshold: " + LINEAR_VELOCITY_THRESHOLD + " ticks/sec");
        writeLog("Angular Velocity Threshold: " + ANGULAR_VELOCITY_THRESHOLD + " deg/sec");
        writeLog("Trials Per Power: " + TRIALS_PER_POWER);
        writeLog("");

        KLog.d(LOG_TAG, () -> String.format("Starting tuning: Power %.3f to %.3f, increment %.3f",
                MIN_POWER, MAX_POWER, POWER_INCREMENT));

        // Main testing loop
        double currentTestPower = MIN_POWER;

        while (opModeIsActive() && currentTestPower <= MAX_POWER) {
            // Test linear movement
            for (int trial = 0; trial < TRIALS_PER_POWER && opModeIsActive(); trial++) {
                testLinearMovement(currentTestPower, trial);
            }

            // Test rotational movement
            for (int trial = 0; trial < TRIALS_PER_POWER && opModeIsActive(); trial++) {
                testRotationalMovement(currentTestPower, trial);
            }

            currentTestPower += POWER_INCREMENT;
        }

        // Calculate and display results
        calculateResults();
        displayResults();

        // Cleanup
        driveTrain.setPower(0.0);
        KLog.d(LOG_TAG, "=== OpMode Complete ===");
        if (fileWriter != null) {
            writeLog("");
            writeLog("=== OpMode Complete ===");
            fileWriter.close();
        }
    }

    private void testLinearMovement(double power, int trial) {
        ElapsedTime timer = new ElapsedTime();

        // Update telemetry
        telemetry.addData("Testing", "Linear Movement");
        telemetry.addData("Power", String.format("%.4f", power));
        telemetry.addData("Trial", (trial + 1) + "/" + TRIALS_PER_POWER);
        telemetry.addData("Progress", String.format("%.1f%%", (power / MAX_POWER) * 100));
        telemetry.update();

        KLog.d(LOG_TAG, () -> String.format("Testing Linear - Power: %.4f, Trial: %d/%d",
                power, trial + 1, TRIALS_PER_POWER));

        // Set all motors to test power
        driveTrain.setPower(power);

        // Wait for settle time
        timer.reset();
        while (timer.seconds() < SETTLE_TIME && opModeIsActive()) {
            sleep(10);
        }

        // Store initial encoder positions
        int rightEncoderStart = rightEncoder.getCurrentPosition();
        int leftEncoderStart = leftEncoder.getCurrentPosition();

        // Measure for test duration
        timer.reset();
        while (timer.seconds() < TEST_DURATION && opModeIsActive()) {
            sleep(10);
        }

        // Calculate velocities
        int rightPosFinal = rightEncoder.getCurrentPosition();
        int leftPosFinal = leftEncoder.getCurrentPosition();
        double duration = timer.seconds();

        double rightVelocity = Math.abs((rightPosFinal - rightEncoderStart) / duration);
        double leftVelocity = Math.abs((leftPosFinal - leftEncoderStart) / duration);
        double avgVelocity = (rightVelocity + leftVelocity) / 2.0;

        boolean moved = avgVelocity > LINEAR_VELOCITY_THRESHOLD;

        String result = String.format("Linear Test - Power: %.4f, Trial: %d, Velocity: %.2f ticks/sec, Moved: %b",
                power, trial + 1, avgVelocity, moved);
        writeLog(result);
        KLog.d(LOG_TAG, () -> result);

        if (moved) {
            linearValidPowers.add(power);
        }

        // Stop and wait
        driveTrain.setPower(0.0);
        sleep(500);
    }

    private void testRotationalMovement(double power, int trial) {
        ElapsedTime timer = new ElapsedTime();

        // Update telemetry
        telemetry.addData("Testing", "Rotational Movement");
        telemetry.addData("Power", String.format("%.4f", power));
        telemetry.addData("Trial", (trial + 1) + "/" + TRIALS_PER_POWER);
        telemetry.addData("Progress", String.format("%.1f%%", (power / MAX_POWER) * 100));
        telemetry.update();

        KLog.d(LOG_TAG, () -> String.format("Testing Rotational - Power: %.4f, Trial: %d/%d",
                power, trial + 1, TRIALS_PER_POWER));

        // Set motors for rotation (left positive, right negative)
        driveTrain.setPower(power, -power, power, -power);

        // Wait for settle time
        timer.reset();
        while (timer.seconds() < SETTLE_TIME && opModeIsActive()) {
            sleep(10);
        }

        // Store initial angle
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double angleStart = angles.getYaw(AngleUnit.DEGREES);

        // Measure for test duration
        timer.reset();
        while (timer.seconds() < TEST_DURATION && opModeIsActive()) {
            sleep(10);
        }

        // Calculate angular velocity
        YawPitchRollAngles anglesFinal = imu.getRobotYawPitchRollAngles();
        double finalAngle = anglesFinal.getYaw(AngleUnit.DEGREES);
        double duration = timer.seconds();

        double angleDelta = Math.abs(finalAngle - angleStart);

        // Handle angle wraparound
        if (angleDelta > 180) {
            angleDelta = 360 - angleDelta;
        }

        double angularVelocity = angleDelta / duration;

        boolean rotated = angularVelocity > ANGULAR_VELOCITY_THRESHOLD;

        String result = String.format("Rotational Test - Power: %.4f, Trial: %d, Angular Velocity: %.2f deg/sec, Rotated: %b",
                power, trial + 1, angularVelocity, rotated);
        writeLog(result);
        KLog.d(LOG_TAG, () -> result);

        if (rotated) {
            rotationalValidPowers.add(power);
        }

        // Stop and wait
        driveTrain.setPower(0.0);
        sleep(500);
    }

    private void calculateResults() {
        // Calculate minimum power for linear movement
        if (!linearValidPowers.isEmpty()) {
            minLinearPower = linearValidPowers.stream()
                    .mapToDouble(Double::doubleValue)
                    .min()
                    .orElse(-1);
        }

        // Calculate minimum power for rotational movement
        if (!rotationalValidPowers.isEmpty()) {
            minRotationalPower = rotationalValidPowers.stream()
                    .mapToDouble(Double::doubleValue)
                    .min()
                    .orElse(-1);
        }

        writeLog("");
        writeLog("=== FINAL RESULTS ===");
        String linearResult = "Minimum Linear Power (kS forward/backward): " +
                (minLinearPower >= 0 ? String.format("%.4f", minLinearPower) : "NOT FOUND");
        String rotationalResult = "Minimum Rotational Power (kS theta): " +
                (minRotationalPower >= 0 ? String.format("%.4f", minRotationalPower) : "NOT FOUND");

        writeLog(linearResult);
        writeLog(rotationalResult);
        writeLog("");
        writeLog("Linear valid powers: " + linearValidPowers.size() + " trials succeeded");
        writeLog("Rotational valid powers: " + rotationalValidPowers.size() + " trials succeeded");

        KLog.d(LOG_TAG, "=== FINAL RESULTS ===");
        KLog.d(LOG_TAG, () -> linearResult);
        KLog.d(LOG_TAG, () -> rotationalResult);
        KLog.d(LOG_TAG, () -> "Linear trials succeeded: " + linearValidPowers.size());
        KLog.d(LOG_TAG, () -> "Rotational trials succeeded: " + rotationalValidPowers.size());
    }

    private void displayResults() {
        KLog.d(LOG_TAG, "Displaying final results");

        telemetry.clear();
        telemetry.addData("=== TUNING COMPLETE ===", "");
        telemetry.addData("", "");
        telemetry.addData("Min Linear Power (kS)", minLinearPower >= 0 ? String.format("%.4f", minLinearPower) : "NOT FOUND");
        telemetry.addData("Min Rotational Power (kS)", minRotationalPower >= 0 ? String.format("%.4f", minRotationalPower) : "NOT FOUND");
        telemetry.addData("", "");
        telemetry.addData("Linear Tests Passed", linearValidPowers.size());
        telemetry.addData("Rotational Tests Passed", rotationalValidPowers.size());
        telemetry.addData("", "");
        telemetry.addData("Log saved to", "OdometryLog/");
        telemetry.update();

        // Keep results displayed
        while (opModeIsActive()) {
            sleep(100);
        }
    }

    private void writeLog(String message) {
        if (fileWriter != null) {
            fileWriter.writeLine(message);
        }
    }
}
