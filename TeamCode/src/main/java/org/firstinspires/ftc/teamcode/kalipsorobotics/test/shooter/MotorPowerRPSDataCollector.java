package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KGamePad;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KMotor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;

/**
 * Measures the flywheel kF and tunes kP, one motor, no balls. GP1: A = kF sequence, B = kP sequence, X = cancel.
 * Run A first: B uses the kF A measured (ShooterConfig.kf if A was not run). Nothing is written into ShooterConfig:
 * copy the two values at the end into it. Pattern source: this file's pre-milestone-2 two-motor version.
 *
 * A, kF: open loop, power POWER_START to POWER_END in POWER_INCREMENT steps. Each step waits STABILIZATION_TIME_MS, then
 *   averages the RPS for DATA_COLLECTION_TIME_MS. kF = effective power / RPS, where effective power = power before
 *   KMotor.voltageScale, matching KMotor.goToRPS. Result = average of the steps.
 *   File MotorPowerRPSData: Power, RPS_MAX, RPS_MIN, RPS_avg, Battery_Voltage, kf
 *
 * B, kP: KP_RPS_START to KP_RPS_END in KP_RPS_STEP steps. Every trial settles at KP_STEP_RPS below the target, then steps up
 *   to it with KMotor.goToRPS (the loop the robot shoots with) at the kP being tried. KpTuner picks the kPs, scores each
 *   trial (settle time, overshoot, oscillating) and verifies the final kP at every RPS.
 *   File MotorKpTuning: Phase, TargetRPS, kP, kF, StartRPS, SettleMs, OvershootRPS, ErrorRPS, Oscillating, Battery_Voltage
 *
 * Every value found stays on screen while running and at the end.
 */

@TeleOp(name = "MotorPowerRPSDataCollector", group = "Test")
public class MotorPowerRPSDataCollector extends LinearOpMode {

    // Test parameters
    private static final double POWER_START = 0.3;
    private static final double POWER_END = 1;
    private static final double POWER_INCREMENT = 0.1;
    private static final long STABILIZATION_TIME_MS = 6000; // 6 s
    private static final long DATA_COLLECTION_TIME_MS = 1000; // 1 second
    private static final long SAMPLE_INTERVAL_MS = 10; // 10 ms
    // kP sequence
    private static final double KP_RPS_START = 21;
    private static final double KP_RPS_END = 56;
    private static final double KP_RPS_STEP = 5;
    private static final double KP_STEP_RPS = 10; // size of the step each trial answers
    private static final double START_TOLERANCE_RPS = 4; // steady this close to the start speed is fine (kP is low there)
    private static final double START_STEADY_RPS = 1.5; // steady = RPS spread over START_STEADY_MS (2 hub velocity steps)
    private static final long START_STEADY_MS = 600;
    private static final long START_TIMEOUT_MS = 20000; // coasting down is slow
    private static final long TRIAL_MAX_MS = 4000;
    private static final long TRIAL_AFTER_REACH_MS = 1500; // keep watching for oscillation
    private static final long LOOP_MS = 20; // about TeleOp's loop time: a faster loop here would allow a kP TeleOp can't hold
    private static final long VOLTAGE_READ_MS = 500; // like KOpMode.updateActions
    private static final String TAG = "Test_Shooter";
    private KMotor shooter;
    private KFileWriter fileWriter;
    private KFileWriter kpWriter;
    private boolean cancelled = false;
    private String message = "";
    private KpTuner kpTuner = null;
    private long lastVoltageReadMs = 0;
    private OpModeUtilities opModeUtilities;
    private final ArrayList<String> stepKfLines = new ArrayList<>();
    private double kfTotal = 0;
    private int kfSteps = 0;
    private boolean kfComplete = false; // a cancelled sequence only has the low powers: not a usable average

    /** kf = effective power / rps; effective power mirrors KMotor.goToRPS voltage compensation. */
    static double computeKf(double power, double batteryVoltage, double rpsAvg) {
        if (rpsAvg <= 0) {
            return 0;
        }
        return power / KMotor.voltageScale(batteryVoltage) / rpsAvg;
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

        kpWriter = new KFileWriter("MotorKpTuning", opModeUtilities);
        kpWriter.writeLine("Phase,TargetRPS,kP,kF,StartRPS,SettleMs,OvershootRPS,ErrorRPS,Oscillating,Battery_Voltage");

        addInstructions();
        telemetry.addLine("Press PLAY, then A or B");
        telemetry.update();

        KLog.d(TAG, "event=initDone");

        waitForStart();

        KGamePad pad = new KGamePad(gamepad1);
        while (opModeIsActive()) {
            boolean a = pad.isButtonAFirstPressed();
            boolean b = pad.isButtonBFirstPressed();
            if (!gamepad1.start && (a || b)) { // Start+A / Start+B only assigns the gamepad
                cancelled = false;
                message = "";
                if (a) {
                    runKfSequence();
                } else {
                    runKpSequence();
                }
                shooter.stopAndResetPID();
                if (cancelled) {
                    message = "CANCELLED with X";
                }
                flush(fileWriter);
                flush(kpWriter);
                pad = new KGamePad(gamepad1);
                pad.isButtonAFirstPressed(); // a button held from the sequence must not start another one
                pad.isButtonBFirstPressed();
            }
            addInstructions();
            if (!message.isEmpty()) {
                telemetry.addLine(message);
            }
            addResults();
            telemetry.update();
            sleep(LOOP_MS);
        }

        shooter.stopAndResetPID();
        fileWriter.close();
        kpWriter.close();
        KLog.d(TAG, "event=done file=closed");
    }

    /** False once the OpMode stops or X is pressed. */
    private boolean running() {
        if (gamepad1.x && !cancelled) {
            cancelled = true;
            KLog.d(TAG, "event=cancel");
        }
        return opModeIsActive() && !cancelled;
    }

    private void runKfSequence() {
        stepKfLines.clear();
        kfTotal = 0;
        kfSteps = 0;
        kfComplete = false;
        double currentPower = POWER_START;
        int testNumber = 1;
        int totalTests = (int) Math.round((POWER_END - POWER_START) / POWER_INCREMENT) + 1;

        while (running() && currentPower <= POWER_END + 0.001) { // +0.001 for floating point precision
            // Step 1: Set power
            shooter.setPower(currentPower);

            double finalCurrentPower = currentPower;
            int finalTestNumber = testNumber;
            KLog.d(TAG, () -> String.format("event=stabilizing test=%d/%d power=%.2f",
                    finalTestNumber, totalTests, finalCurrentPower));

            // Step 2: Wait for stabilization
            long stableAtMs = System.currentTimeMillis() + STABILIZATION_TIME_MS;
            while (running() && System.currentTimeMillis() < stableAtMs) {
                double rps = shooter.getRPS();
                telemetry.addLine("=== kF sequence (X = cancel) ===");
                telemetry.addData("Test", "%d / %d", testNumber, totalTests);
                telemetry.addData("Current Power", "%.2f", currentPower);
                telemetry.addData("Status", "spinning up, %.1f s left", (stableAtMs - System.currentTimeMillis()) / 1000.0);
                telemetry.addData("Shooter Current RPS", "%.2f", rps);
                telemetry.addData("kF now (power / (RPS * voltageScale))", "%.5f", computeKf(currentPower, getBatteryVoltage(), rps));
                addResults();
                telemetry.update();
                sleep(LOOP_MS);
            }

            // Step 3: Collect RPS data for 1 second. Voltage read under load, as goToRPS sees it.
            double batteryVoltage = getBatteryVoltage();
            ArrayList<Double> rpsData = new ArrayList<>();
            long startTime = System.currentTimeMillis();
            long endTime = startTime + DATA_COLLECTION_TIME_MS;

            while (System.currentTimeMillis() < endTime && running()) {
                double rps = shooter.getRPS();
                rpsData.add(rps);

                telemetry.addLine("=== kF sequence (X = cancel) ===");
                telemetry.addData("Test", "%d / %d", testNumber, totalTests);
                telemetry.addData("Current Power", "%.2f", currentPower);
                telemetry.addLine("Status: Collecting RPS data...");
                telemetry.addData("Samples Collected", rpsData.size());
                telemetry.addData("Shooter Current RPS", "%.2f", rps);
                telemetry.addData("kF now (power / (RPS * voltageScale))", "%.5f", computeKf(currentPower, batteryVoltage, rps));
                addResults();
                telemetry.update();

                sleep(SAMPLE_INTERVAL_MS);
            }

            if (!running()) {
                return; // a cut-off step is not a measurement
            }

            // Step 4: Calculate statistics
            double rpsMax = calculateMax(rpsData);
            double rpsMin = calculateMin(rpsData);
            double rpsAvg = calculateAverage(rpsData);
            double kf = computeKf(currentPower, batteryVoltage, rpsAvg);
            stepKfLines.add(String.format("%.2f power: %.2f RPS  kF %.5f  (%.2f V)", currentPower, rpsAvg, kf, batteryVoltage));
            if (kf > 0) { // 0 = the flywheel did not move
                kfTotal += kf;
                kfSteps++;
            }

            fileWriter.writeLine(String.format("%.2f,%.2f,%.2f,%.2f,%.2f,%.5f",
                currentPower, rpsMax, rpsMin, rpsAvg, batteryVoltage, kf));

            int finalTestNumber1 = testNumber;
            double finalCurrentPower1 = currentPower;
            KLog.d(TAG, () -> String.format(
                "event=testDone test=%d/%d power=%.2f rpsMax=%.2f rpsMin=%.2f rpsAvg=%.2f kf=%.5f batteryV=%.2f",
                    finalTestNumber1, totalTests, finalCurrentPower1, rpsMax, rpsMin, rpsAvg, kf, batteryVoltage));

            // Move to next power level
            currentPower += POWER_INCREMENT;
            testNumber++;
        }

        double kfAverage = averageKf();
        if (running()) {
            kfComplete = kfSteps > 0;
            message = "kF SEQUENCE COMPLETE. Press B to tune kP with this kF.";
        }
        KLog.d(TAG, () -> String.format("event=kfAverage kf=%.5f steps=%d", kfAverage, kfSteps));
    }

    private double averageKf() {
        return kfSteps > 0 ? kfTotal / kfSteps : Double.NaN;
    }

    /** The kF the kP sequence runs with. */
    private double tuningKf() {
        return kfComplete ? averageKf() : ShooterConfig.kf;
    }

    private void runKpSequence() {
        double kf = tuningKf();
        double[] targets = new double[(int) Math.round((KP_RPS_END - KP_RPS_START) / KP_RPS_STEP) + 1];
        for (int i = 0; i < targets.length; i++) {
            targets[i] = KP_RPS_START + i * KP_RPS_STEP;
        }
        kpTuner = new KpTuner(TAG, kf, targets, (target, kp) -> runKpTrial(target, kp, kf));
        KLog.d(TAG, () -> String.format("event=kpStart kf=%.5f", kf));
        if (kpTuner.run()) {
            message = "kP SEQUENCE COMPLETE: " + kpTuner.getStatus();
        }
    }

    /** Settles at target - KP_STEP_RPS, then records the step up to target at this kP. null = cancelled or no start. */
    private KpTuner.Trial runKpTrial(double target, double kp, double kf) {
        String phase = kpTuner.getVerify().isEmpty() && Double.isNaN(kpTuner.getBestKp()) ? "tune" : "verify";
        double startTarget = target - KP_STEP_RPS;
        shooter.getPIDFController().setKf(kf);
        shooter.getPIDFController().setKp(kf); // same calm kP for every start
        shooter.resetPID();
        ArrayList<Double> times = new ArrayList<>(), rpsData = new ArrayList<>();
        double startRps = 0;
        boolean started = false;
        long t0 = System.currentTimeMillis();
        while (!started) {
            if (!running()) {
                return null;
            }
            long now = System.currentTimeMillis() - t0;
            if (now > START_TIMEOUT_MS) {
                double at = startRps;
                message = String.format("kP STOPPED: no steady %.0f RPS start in %d s (at %.1f). Is kF right? Run A first.",
                        startTarget, START_TIMEOUT_MS / 1000, at);
                KLog.e(TAG, "event=kpNoStart " + message);
                return null;
            }
            refreshVoltage();
            startRps = shooter.goToRPS(startTarget);
            times.add((double) now);
            rpsData.add(startRps);
            double min = startRps, max = startRps;
            for (int i = times.size() - 1; i >= 0 && now - times.get(i) <= START_STEADY_MS; i--) {
                min = Math.min(min, rpsData.get(i));
                max = Math.max(max, rpsData.get(i));
            }
            started = now > START_STEADY_MS && max - min <= START_STEADY_RPS && Math.abs(startRps - startTarget) <= START_TOLERANCE_RPS;
            kpTelemetry(phase + ": going to the start speed", target, kp, startTarget, startRps);
            sleep(LOOP_MS);
        }

        shooter.getPIDFController().setKp(kp);
        shooter.resetPID();
        times.clear();
        rpsData.clear();
        long reachedMs = -1;
        t0 = System.currentTimeMillis();
        while (true) {
            if (!running()) {
                return null;
            }
            long now = System.currentTimeMillis() - t0;
            if (now > TRIAL_MAX_MS || (reachedMs >= 0 && now - reachedMs > TRIAL_AFTER_REACH_MS)) {
                break;
            }
            refreshVoltage();
            double rps = shooter.goToRPS(target);
            times.add((double) now);
            rpsData.add(rps);
            if (reachedMs < 0 && rps >= target - KpTuner.TOLERANCE_RPS) {
                reachedMs = now;
            }
            kpTelemetry(phase + ": step", target, kp, target, rps);
            sleep(LOOP_MS);
        }
        KpTuner.Trial trial = KpTuner.analyze(target, kp, times, rpsData);
        kpWriter.writeLine(String.format("%s,%.1f,%.5f,%.5f,%.2f,%.0f,%.2f,%.2f,%d,%.2f", phase, target, kp, kf, startRps,
                trial.settleMs, trial.overshootRps, trial.errorRps, trial.oscillating ? 1 : 0, SharedData.getVoltage()));
        return trial;
    }

    /** goToRPS compensates with SharedData's voltage; this OpMode has no KOpMode loop to refresh it. */
    private void refreshVoltage() {
        long now = System.currentTimeMillis();
        if (now - lastVoltageReadMs > VOLTAGE_READ_MS) {
            SharedData.setVoltage(getBatteryVoltage());
            lastVoltageReadMs = now;
        }
    }

    private void kpTelemetry(String status, double target, double kp, double commanded, double rps) {
        telemetry.addLine("=== kP sequence (X = cancel) ===");
        telemetry.addData("Status", "%s  (%s)", status, kpTuner.getStatus());
        telemetry.addData("Target RPS / kP being tried", "%.0f / %.5f", target, kp);
        telemetry.addData("Commanded / current RPS / power", "%.1f / %.2f / %.3f", commanded, rps, shooter.getPower());
        addResults();
        telemetry.update();
    }

    private void addInstructions() {
        telemetry.addLine("=== Shooter kF / kP tuner (GP1) ===");
        telemetry.addLine("No balls, stopper closed, full battery. The flywheel spins by itself.");
        telemetry.addLine(String.format("A: kF sequence, power %.1f to %.1f by %.1f, ~1 min. Run this first.", POWER_START, POWER_END, POWER_INCREMENT));
        telemetry.addLine(String.format("B: kP sequence, %.0f to %.0f RPS by %.0f, several minutes. Uses kF %.5f (%s)",
                KP_RPS_START, KP_RPS_END, KP_RPS_STEP, tuningKf(), kfComplete ? "from A" : "ShooterConfig, A not finished"));
        telemetry.addLine("X: cancel the running sequence (hold briefly)");
        telemetry.addLine("At the end copy FINAL kF and BEST kP into ShooterConfig. Files: RobotLogs/MotorPowerRPSData, MotorKpTuning");
        telemetry.addLine();
    }

    /** Every kF and kP value found so far, finals first; shown before each telemetry.update() so they stay on screen. */
    private void addResults() {
        telemetry.addData(kfComplete ? "FINAL AVERAGE kF" : "Average kF so far", kfSteps > 0 ? String.format("%.5f (%d steps)", averageKf(), kfSteps) : "not measured");
        if (kpTuner != null) {
            telemetry.addData("BEST kP", Double.isNaN(kpTuner.getBestKp()) ? kpTuner.getStatus()
                    : String.format("%.5f  %s  (with kF %.5f, average of RPS bests %.5f)", kpTuner.getBestKp(),
                            kpTuner.isVerified() ? "VERIFIED" : "not verified: " + kpTuner.getStatus(), kpTuner.getKf(), kpTuner.getAverageKp()));
        } else {
            telemetry.addData("BEST kP", "not tuned");
        }
        telemetry.addData("In ShooterConfig now", "kF %.5f  kP %.5f", ShooterConfig.kf, ShooterConfig.kp);
        telemetry.addLine("--- kF per step ---");
        for (String line : stepKfLines) {
            telemetry.addLine(line);
        }
        if (kpTuner != null) {
            telemetry.addLine("--- kP per RPS (kP, settle time; OSC = oscillated) ---");
            for (KpTuner.Target target : kpTuner.getTargets()) {
                telemetry.addLine(target.bestText());
                telemetry.addLine(target.trialsText());
            }
            if (!kpTuner.getVerify().isEmpty()) {
                telemetry.addLine("--- BEST kP at each RPS (verify) ---");
                for (KpTuner.Trial t : kpTuner.getVerify()) {
                    telemetry.addLine(String.format("%.0f RPS: kP %s  overshoot %.1f  error %.1f", t.targetRps, t.shortText(), t.overshootRps, t.errorRps));
                }
            }
        }
    }

    private static void flush(KFileWriter writer) {
        try {
            writer.flush();
        } catch (java.io.IOException e) {
            KLog.e(TAG, "event=flushFailed", e);
        }
    }

    /**
     * Initialize the shooter motor with proper configuration
     */
    private void initializeHardware() {
        DcMotor motor = hardwareMap.dcMotor.get("shooter");
        motor.setDirection(ShooterConfig.SHOOTER_MOTOR_DIRECTION); // same constant Shooter uses
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
