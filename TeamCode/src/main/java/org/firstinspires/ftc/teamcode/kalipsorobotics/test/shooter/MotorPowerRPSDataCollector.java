package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KGamePad;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KMotor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;

/**
 * Measures kF and tunes kP for the flywheel and for each intake motor (A = intakeFront, B = intakeBack). GP1:
 *   Shooter:  A = kF sequence,           B = kP sequence (no balls)
 *   Intake A: DPAD RIGHT = kF sequence,  DPAD UP = kP sequence     (intake B runs open loop at OTHER_INTAKE_POWER)
 *   Intake B: DPAD LEFT = kF sequence,   DPAD DOWN = kP sequence   (intake A runs open loop at OTHER_INTAKE_POWER)
 *   X = cancel
 * The stopper is held closed the whole OpMode. Intake: load the balls before pressing. Each press is one sequence on one motor; the screen then shows that motor's results.
 * Run a motor's kF first: its kP sequence uses the kF that measured (the config kF if it was not run). Nothing is written
 * into the configs: copy the values at the end into ShooterConfig.kf / kp or IntakeConfig.SIDE_A.kF / kP (SIDE_B).
 * Pattern source: this file's pre-milestone-2 two-motor version.
 *
 * kF: open loop, power POWER_START to POWER_END in POWER_INCREMENT steps. Each step waits the motor's stabilization time,
 *   then averages the RPS for DATA_COLLECTION_TIME_MS. kF = effective power / RPS, where effective power = power before
 *   KMotor.voltageScale, matching KMotor.goToRPS and IntakeModule.runRPS. Result = average of the steps.
 *   Files MotorPowerRPSData (shooter), IntakeAPowerRPSData, IntakeBPowerRPSData:
 *   Power, RPS_MAX, RPS_MIN, RPS_avg, Battery_Voltage, kf
 *
 * kP: one target RPS at a time, lowest first. Every trial settles at the motor's step size below the target, then steps
 *   up to it with the loop the robot runs (shooter: KMotor.goToRPS; intake: IntakeModule.runRPS) at the kP being tried.
 *   KpTuner raises / bisects kP at that RPS until it has the fastest calm (accurate, not oscillating) one, then moves to
 *   the next RPS, and at the end verifies one kP at every RPS.
 *   Shooter: SHOOTER_KP_RPS_START to SHOOTER_KP_RPS_END. Intake: IntakeConfig.FEED_RPS minus INTAKE_KP_BELOW_FEED_RPS
 *   (the feed runs near FEED_RPS; loaded with balls, the 1620 rpm motor tops out well below its ~27 RPS free speed).
 *   Files MotorKpTuning (shooter), IntakeAKpTuning, IntakeBKpTuning:
 *   Phase, TargetRPS, kP, kF, StartRPS, SettleMs, OvershootRPS, ErrorRPS, Oscillating, Battery_Voltage
 *
 * Every value found stays on screen while running and at the end.
 */

@TeleOp(name = "MotorPowerRPSDataCollector", group = "Test")
public class MotorPowerRPSDataCollector extends LinearOpMode {

    // kF sequence, every motor
    private static final double POWER_START = 0.4;
    private static final double POWER_END = 1;
    private static final double POWER_INCREMENT = 0.1;
    private static final long DATA_COLLECTION_TIME_MS = 1000; // 1 second
    private static final long SAMPLE_INTERVAL_MS = 10; // 10 ms
    // kP sequence, every motor
    private static final long START_STEADY_MS = 600;
    private static final long TRIAL_MAX_MS = 4000;
    private static final long TRIAL_AFTER_REACH_MS = 1500; // keep watching for oscillation
    private static final long LOOP_MS = 20; // about TeleOp's loop time: a faster loop here would allow a kP TeleOp can't hold
    private static final long VOLTAGE_READ_MS = 500; // like KOpMode.updateActions
    // Shooter
    private static final long SHOOTER_STABILIZATION_MS = 6000; // 6 s
    private static final double SHOOTER_KP_RPS_START = 21;
    private static final double SHOOTER_KP_RPS_END = 56;
    private static final double SHOOTER_KP_RPS_STEP = 5;
    private static final double SHOOTER_KP_STEP_RPS = 10; // size of the step each trial answers
    private static final double SHOOTER_START_TOLERANCE_RPS = 4; // steady this close to the start speed is fine (kP is low there)
    private static final double SHOOTER_START_STEADY_RPS = 1.5; // steady = RPS spread over START_STEADY_MS (2 hub velocity steps)
    private static final long SHOOTER_START_TIMEOUT_MS = 20000; // coasting down is slow
    // Intake, in transfer motor RPS (ModuleConfig.INTAKE_TICKS_PER_REV; hub velocity steps are ~0.2 RPS)
    private static final double OTHER_INTAKE_POWER = 0.7; // the side not being tuned runs open loop at this the whole sequence
    private static final long INTAKE_STABILIZATION_MS = 1500; // light wheels: at speed well under a second
    private static final double[] INTAKE_KP_BELOW_FEED_RPS = {3, 1.5, 0}; // targets = FEED_RPS - these, lowest first
    private static final double INTAKE_KP_STEP_RPS = 3;
    private static final double INTAKE_MIN_START_RPS = 1; // slower than this the wheels may stick
    // Loose start checks: balls loading the wheels make the speed jump, and a start that never counts as steady stops the sequence.
    private static final double INTAKE_START_TOLERANCE_RPS = 1.5;
    private static final double INTAKE_START_STEADY_RPS = 1.5;
    private static final long INTAKE_START_TIMEOUT_MS = 8000;

    private static final String TAG = "Test_Shooter";
    private OpModeUtilities opModeUtilities;
    private Tuned shooter;
    private Tuned intakeA;
    private Tuned intakeB;
    private Stopper stopper;
    private boolean cancelled = false;
    private final ArrayList<String> messages = new ArrayList<>(); // results / problems of the last sequence, one line each
    private long lastVoltageReadMs = 0;

    /** kf = effective power / rps; effective power mirrors KMotor.goToRPS voltage compensation. */
    static double computeKf(double power, double batteryVoltage, double rpsAvg) {
        if (rpsAvg <= 0) {
            return 0;
        }
        return power / KMotor.voltageScale(batteryVoltage) / rpsAvg;
    }

    /** One motor: how to drive it, its sequence settings, and what was found. */
    private abstract class Tuned {
        final String name;
        final String tag;
        final String kfButton;
        final String kpButton;
        final String configNames; // where the results get copied
        final String kfFileName;
        final String kpFileName;
        final long stabilizationMs;
        final double kpStepRps;
        final double startToleranceRps;
        final double startSteadyRps;
        final long startTimeoutMs;

        final ArrayList<String> stepKfLines = new ArrayList<>();
        double kfTotal = 0;
        int kfSteps = 0;
        boolean kfComplete = false; // a cancelled sequence only has the low powers: not a usable average
        KpTuner kpTuner = null;
        private KFileWriter kfWriter = null;
        private KFileWriter kpWriter = null;

        Tuned(String name, String tag, String kfButton, String kpButton, String configNames, String kfFileName, String kpFileName,
              long stabilizationMs, double kpStepRps, double startToleranceRps, double startSteadyRps, long startTimeoutMs) {
            this.name = name;
            this.tag = tag;
            this.kfButton = kfButton;
            this.kpButton = kpButton;
            this.configNames = configNames;
            this.kfFileName = kfFileName;
            this.kpFileName = kpFileName;
            this.stabilizationMs = stabilizationMs;
            this.kpStepRps = kpStepRps;
            this.startToleranceRps = startToleranceRps;
            this.startSteadyRps = startSteadyRps;
            this.startTimeoutMs = startTimeoutMs;
        }

        abstract void setPower(double power);

        abstract double getRPS();

        abstract double getPower();

        /** One closed-loop update with these gains; returns the RPS it read. */
        abstract double goToRPS(double targetRps, double kp, double kf);

        /** Clears the loop's memory before a new target. */
        abstract void resetLoop();

        abstract void stop();

        abstract double configKf();

        abstract double configKp();

        abstract double[] kpTargets();

        /** Starts / stops whatever else runs during this motor's sequences (the other intake side). */
        void startOther() {
        }

        void stopOther() {
        }

        /** Telemetry line for whatever else runs during the sequence; null = nothing. */
        String otherStatus() {
            return null;
        }

        double averageKf() {
            return kfSteps > 0 ? kfTotal / kfSteps : Double.NaN;
        }

        /** The kF the kP sequence runs with. */
        double tuningKf() {
            return kfComplete ? averageKf() : configKf();
        }

        KFileWriter kfWriter() {
            if (kfWriter == null) {
                kfWriter = new KFileWriter(kfFileName, opModeUtilities);
                kfWriter.writeLine("Power,RPS_MAX,RPS_MIN,RPS_avg,Battery_Voltage,kf");
            }
            return kfWriter;
        }

        KFileWriter kpWriter() {
            if (kpWriter == null) {
                kpWriter = new KFileWriter(kpFileName, opModeUtilities);
                kpWriter.writeLine("Phase,TargetRPS,kP,kF,StartRPS,SettleMs,OvershootRPS,ErrorRPS,Oscillating,Battery_Voltage");
            }
            return kpWriter;
        }

        void flushFiles() {
            flush(kfWriter);
            flush(kpWriter);
        }

        void closeFiles() {
            if (kfWriter != null) {
                kfWriter.close();
            }
            if (kpWriter != null) {
                kpWriter.close();
            }
        }
    }

    private class ShooterTuned extends Tuned {
        private final KMotor motor;

        ShooterTuned(KMotor motor) {
            super("Shooter", TAG, "A", "B", "ShooterConfig.kf / kp", "MotorPowerRPSData", "MotorKpTuning",
                    SHOOTER_STABILIZATION_MS, SHOOTER_KP_STEP_RPS, SHOOTER_START_TOLERANCE_RPS, SHOOTER_START_STEADY_RPS,
                    SHOOTER_START_TIMEOUT_MS);
            this.motor = motor;
        }

        @Override
        void setPower(double power) {
            motor.setPower(power);
        }

        @Override
        double getRPS() {
            return motor.getRPS();
        }

        @Override
        double getPower() {
            return motor.getPower();
        }

        @Override
        double goToRPS(double targetRps, double kp, double kf) {
            motor.getPIDFController().setKf(kf);
            motor.getPIDFController().setKp(kp);
            return motor.goToRPS(targetRps);
        }

        @Override
        void resetLoop() {
            motor.resetPID();
        }

        @Override
        void stop() {
            motor.stopAndResetPID();
        }

        @Override
        double configKf() {
            return ShooterConfig.kf;
        }

        @Override
        double configKp() {
            return ShooterConfig.kp;
        }

        @Override
        double[] kpTargets() {
            double[] targets = new double[(int) Math.round((SHOOTER_KP_RPS_END - SHOOTER_KP_RPS_START) / SHOOTER_KP_RPS_STEP) + 1];
            for (int i = 0; i < targets.length; i++) {
                targets[i] = SHOOTER_KP_RPS_START + i * SHOOTER_KP_RPS_STEP;
            }
            return targets;
        }
    }

    private class IntakeTuned extends Tuned {
        private final IntakeModule side;
        private final IntakeModule other; // runs open loop at OTHER_INTAKE_POWER during this side's sequences

        IntakeTuned(IntakeModule side, IntakeModule other, String kfButton, String kpButton) {
            super("Intake " + side.getSide(), "Test_Intake" + side.getSide(), kfButton, kpButton,
                    "IntakeConfig.SIDE_" + side.getSide() + ".kF / kP",
                    "Intake" + side.getSide() + "PowerRPSData", "Intake" + side.getSide() + "KpTuning",
                    INTAKE_STABILIZATION_MS, INTAKE_KP_STEP_RPS, INTAKE_START_TOLERANCE_RPS, INTAKE_START_STEADY_RPS,
                    INTAKE_START_TIMEOUT_MS);
            this.side = side;
            this.other = other;
        }

        @Override
        void setPower(double power) {
            side.run(power);
        }

        @Override
        double getRPS() {
            return side.getRPS();
        }

        @Override
        double getPower() {
            return side.getPower();
        }

        @Override
        double goToRPS(double targetRps, double kp, double kf) {
            return side.runRPS(targetRps, kf, kp);
        }

        @Override
        void resetLoop() {
            // kF + kP only: the loop has no memory to clear
        }

        @Override
        void stop() {
            side.stop();
        }

        @Override
        void startOther() {
            other.run(OTHER_INTAKE_POWER);
            KLog.d(tag, () -> String.format("event=otherSideOn side=%s power=%.2f", other.getSide(), OTHER_INTAKE_POWER));
        }

        @Override
        void stopOther() {
            other.stop();
        }

        @Override
        String otherStatus() {
            return String.format("Intake %s: open loop %.2f power, %.2f RPS", other.getSide(), other.getPower(), other.getRPS());
        }

        @Override
        double configKf() {
            return side.getConfig().kF;
        }

        @Override
        double configKp() {
            return side.getConfig().kP;
        }

        /** Read when the sequence starts, so a Dashboard FEED_RPS edit applies. Targets whose start would be too slow are left out. */
        @Override
        double[] kpTargets() {
            ArrayList<Double> targets = new ArrayList<>();
            for (double below : INTAKE_KP_BELOW_FEED_RPS) {
                double target = IntakeConfig.FEED_RPS - below;
                if (target - INTAKE_KP_STEP_RPS >= INTAKE_MIN_START_RPS) {
                    targets.add(target);
                }
            }
            double[] result = new double[targets.size()];
            for (int i = 0; i < result.length; i++) {
                result[i] = targets.get(i);
            }
            return result;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("=== Motor Power RPS Data Collector ===");
        telemetry.addLine("Initializing...");
        telemetry.update();

        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        shooter = new ShooterTuned(initializeShooter());
        stopper = new Stopper(opModeUtilities); // constructor closes it; never opened here, so the balls stay in
        IntakeModule sideA = new IntakeModule(opModeUtilities, IntakeSide.FRONT, IntakeConfig.SIDE_A);
        IntakeModule sideB = new IntakeModule(opModeUtilities, IntakeSide.BACK, IntakeConfig.SIDE_B);
        // IntakeModule inits raised (starting size); tuning runs with both sides down, like intaking / feeding.
        sideA.lower();
        sideB.lower();
        intakeA = new IntakeTuned(sideA, sideB, "DPAD RIGHT", "DPAD UP");
        intakeB = new IntakeTuned(sideB, sideA, "DPAD LEFT", "DPAD DOWN");
        Tuned[] all = {shooter, intakeA, intakeB};
        Tuned shown = null; // the motor of the last sequence: its results are on screen

        addInstructions(all);
        telemetry.addLine("Press PLAY, then a sequence button");
        telemetry.update();

        KLog.d(TAG, "event=initDone");

        waitForStart();

        KGamePad pad = new KGamePad(gamepad1);
        while (opModeIsActive()) {
            // Read each once per loop: FirstPressed clears itself.
            boolean a = pad.isButtonAFirstPressed();
            boolean b = pad.isButtonBFirstPressed();
            boolean up = pad.isDpadUpFirstPressed();
            boolean down = pad.isDpadDownFirstPressed();
            boolean left = pad.isDpadLeftFirstPressed();
            boolean right = pad.isDpadRightFirstPressed();
            Tuned motor = a || b ? shooter : up || right ? intakeA : down || left ? intakeB : null;
            boolean kp = b || up || down;
            if (!gamepad1.start && motor != null) { // Start+A / Start+B only assigns the gamepad
                cancelled = false;
                messages.clear();
                shown = motor;
                stopper.close();
                motor.startOther();
                if (kp) {
                    runKpSequence(motor);
                } else {
                    runKfSequence(motor);
                }
                motor.stop();
                motor.stopOther();
                if (cancelled) {
                    messages.add(motor.name + " CANCELLED with X");
                }
                motor.flushFiles();
                pad = new KGamePad(gamepad1);
                clearPresses(pad); // a button held from the sequence must not start another one
            }
            if (shown == null) {
                addInstructions(all);
                for (Tuned t : all) {
                    addSummary(t);
                }
            } else {
                for (String line : messages) {
                    telemetry.addLine(line);
                }
                addSummary(shown);
                addDetails(shown);
                telemetry.addLine();
                addInstructions(all);
            }
            telemetry.update();
            sleep(LOOP_MS);
        }

        for (Tuned t : all) {
            t.stop();
            t.closeFiles();
        }
        KLog.d(TAG, "event=done file=closed");
    }

    private static void clearPresses(KGamePad pad) {
        pad.isButtonAFirstPressed();
        pad.isButtonBFirstPressed();
        pad.isDpadUpFirstPressed();
        pad.isDpadDownFirstPressed();
        pad.isDpadLeftFirstPressed();
        pad.isDpadRightFirstPressed();
    }

    /** False once the OpMode stops or X is pressed. */
    private boolean running() {
        if (gamepad1.x && !cancelled) {
            cancelled = true;
            KLog.d(TAG, "event=cancel");
        }
        return opModeIsActive() && !cancelled;
    }

    private void runKfSequence(Tuned t) {
        t.stepKfLines.clear();
        t.kfTotal = 0;
        t.kfSteps = 0;
        t.kfComplete = false;
        double currentPower = POWER_START;
        int testNumber = 1;
        int totalTests = (int) Math.round((POWER_END - POWER_START) / POWER_INCREMENT) + 1;
        String title = "=== " + t.name + " kF sequence (X = cancel) ===";

        while (running() && currentPower <= POWER_END + 0.001) { // +0.001 for floating point precision
            // Step 1: Set power
            t.setPower(currentPower);

            double finalCurrentPower = currentPower;
            int finalTestNumber = testNumber;
            KLog.d(t.tag, () -> String.format("event=stabilizing test=%d/%d power=%.2f",
                    finalTestNumber, totalTests, finalCurrentPower));

            // Step 2: Wait for stabilization
            long stableAtMs = System.currentTimeMillis() + t.stabilizationMs;
            while (running() && System.currentTimeMillis() < stableAtMs) {
                double rps = t.getRPS();
                telemetry.addLine(title);
                addOtherStatus(t);
                telemetry.addData("Test", "%d / %d", testNumber, totalTests);
                telemetry.addData("Current Power", "%.2f", currentPower);
                telemetry.addData("Status", "spinning up, %.1f s left", (stableAtMs - System.currentTimeMillis()) / 1000.0);
                telemetry.addData(t.name + " Current RPS", "%.2f", rps);
                telemetry.addData("kF now (power / (RPS * voltageScale))", "%.5f", computeKf(currentPower, getBatteryVoltage(), rps));
                addSummary(t);
                addDetails(t);
                telemetry.update();
                sleep(LOOP_MS);
            }

            // Step 3: Collect RPS data for 1 second. Voltage read under load, as the closed loop sees it.
            double batteryVoltage = getBatteryVoltage();
            ArrayList<Double> rpsData = new ArrayList<>();
            long startTime = System.currentTimeMillis();
            long endTime = startTime + DATA_COLLECTION_TIME_MS;

            while (System.currentTimeMillis() < endTime && running()) {
                double rps = t.getRPS();
                rpsData.add(rps);

                telemetry.addLine(title);
                addOtherStatus(t);
                telemetry.addData("Test", "%d / %d", testNumber, totalTests);
                telemetry.addData("Current Power", "%.2f", currentPower);
                telemetry.addLine("Status: Collecting RPS data...");
                telemetry.addData("Samples Collected", rpsData.size());
                telemetry.addData(t.name + " Current RPS", "%.2f", rps);
                telemetry.addData("kF now (power / (RPS * voltageScale))", "%.5f", computeKf(currentPower, batteryVoltage, rps));
                addSummary(t);
                addDetails(t);
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
            t.stepKfLines.add(String.format("%.2f power: %.2f RPS  kF %.5f  (%.2f V)", currentPower, rpsAvg, kf, batteryVoltage));
            if (kf > 0) { // 0 = the motor did not move
                t.kfTotal += kf;
                t.kfSteps++;
            }

            t.kfWriter().writeLine(String.format("%.2f,%.2f,%.2f,%.2f,%.2f,%.5f",
                currentPower, rpsMax, rpsMin, rpsAvg, batteryVoltage, kf));

            int finalTestNumber1 = testNumber;
            double finalCurrentPower1 = currentPower;
            KLog.d(t.tag, () -> String.format(
                "event=testDone test=%d/%d power=%.2f rpsMax=%.2f rpsMin=%.2f rpsAvg=%.2f kf=%.5f batteryV=%.2f",
                    finalTestNumber1, totalTests, finalCurrentPower1, rpsMax, rpsMin, rpsAvg, kf, batteryVoltage));

            // Move to next power level
            currentPower += POWER_INCREMENT;
            testNumber++;
        }

        double kfAverage = t.averageKf();
        if (running()) {
            t.kfComplete = t.kfSteps > 0;
            messages.add(t.name + " kF SEQUENCE COMPLETE. Press " + t.kpButton + " to tune kP with this kF.");
        }
        KLog.d(t.tag, () -> String.format("event=kfAverage kf=%.5f steps=%d", kfAverage, t.kfSteps));
    }

    private void runKpSequence(Tuned t) {
        double kf = t.tuningKf();
        double[] targets = t.kpTargets();
        if (targets.length == 0) {
            String m = String.format("%s kP NOT RUN: IntakeConfig.FEED_RPS %.1f is too low to step %.1f RPS up to it",
                    t.name, IntakeConfig.FEED_RPS, t.kpStepRps);
            messages.add(m);
            KLog.e(t.tag, "event=kpNoTargets " + m);
            return;
        }
        t.kpTuner = new KpTuner(t.tag, kf, targets, (target, kp) -> runKpTrial(t, target, kp, kf));
        KLog.d(t.tag, () -> String.format("event=kpStart kf=%.5f", kf));
        if (t.kpTuner.run()) {
            messages.add(t.name + " kP SEQUENCE COMPLETE: " + t.kpTuner.getStatus());
        }
    }

    /** Settles at target - kpStepRps, then records the step up to target at this kP. null = cancelled or no start. */
    private KpTuner.Trial runKpTrial(Tuned t, double target, double kp, double kf) {
        KpTuner kpTuner = t.kpTuner;
        String phase = kpTuner.getVerify().isEmpty() && Double.isNaN(kpTuner.getBestKp()) ? "tune" : "verify";
        double startTarget = target - t.kpStepRps;
        t.resetLoop();
        ArrayList<Double> times = new ArrayList<>(), rpsData = new ArrayList<>();
        double startRps = 0;
        boolean started = false;
        long t0 = System.currentTimeMillis();
        while (!started) {
            if (!running()) {
                return null;
            }
            long now = System.currentTimeMillis() - t0;
            if (now > t.startTimeoutMs) {
                double at = startRps;
                String m = String.format("%s kP STOPPED: no steady %.1f RPS start in %d s (at %.1f). Is kF right? Run %s first.",
                        t.name, startTarget, t.startTimeoutMs / 1000, at, t.kfButton);
                messages.add(m);
                KLog.e(t.tag, "event=kpNoStart " + m);
                return null;
            }
            refreshVoltage();
            startRps = t.goToRPS(startTarget, kf, kf); // kP = kF: the same calm kP for every start
            times.add((double) now);
            rpsData.add(startRps);
            double min = startRps, max = startRps;
            for (int i = times.size() - 1; i >= 0 && now - times.get(i) <= START_STEADY_MS; i--) {
                min = Math.min(min, rpsData.get(i));
                max = Math.max(max, rpsData.get(i));
            }
            started = now > START_STEADY_MS && max - min <= t.startSteadyRps && Math.abs(startRps - startTarget) <= t.startToleranceRps;
            kpTelemetry(t, phase + ": going to the start speed", target, kp, startTarget, startRps);
            sleep(LOOP_MS);
        }

        t.resetLoop();
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
            double rps = t.goToRPS(target, kp, kf);
            times.add((double) now);
            rpsData.add(rps);
            if (reachedMs < 0 && rps >= target - KpTuner.TOLERANCE_RPS) {
                reachedMs = now;
            }
            kpTelemetry(t, phase + ": step", target, kp, target, rps);
            sleep(LOOP_MS);
        }
        KpTuner.Trial trial = KpTuner.analyze(target, kp, times, rpsData);
        t.kpWriter().writeLine(String.format("%s,%.1f,%.5f,%.5f,%.2f,%.0f,%.2f,%.2f,%d,%.2f", phase, target, kp, kf, startRps,
                trial.settleMs, trial.overshootRps, trial.errorRps, trial.oscillating ? 1 : 0, SharedData.getVoltage()));
        return trial;
    }

    /** goToRPS / runRPS compensate with SharedData's voltage; this OpMode has no KOpMode loop to refresh it. */
    private void refreshVoltage() {
        long now = System.currentTimeMillis();
        if (now - lastVoltageReadMs > VOLTAGE_READ_MS) {
            SharedData.setVoltage(getBatteryVoltage());
            lastVoltageReadMs = now;
        }
    }

    private void kpTelemetry(Tuned t, String status, double target, double kp, double commanded, double rps) {
        telemetry.addLine("=== " + t.name + " kP sequence (X = cancel) ===");
        addOtherStatus(t);
        telemetry.addData("Status", "%s  (%s)", status, t.kpTuner.getStatus());
        telemetry.addData("Target RPS / kP being tried", "%.1f / %.5f", target, kp);
        telemetry.addData("Commanded / current RPS / power", "%.1f / %.2f / %.3f", commanded, rps, t.getPower());
        addSummary(t);
        addDetails(t);
        telemetry.update();
    }

    private void addOtherStatus(Tuned t) {
        String other = t.otherStatus();
        if (other != null) {
            telemetry.addLine(other);
        }
    }

    private void addInstructions(Tuned[] all) {
        telemetry.addLine("=== kF / kP tuner (GP1) ===");
        telemetry.addLine("Full battery. The motors spin by themselves. Shooter: no balls. Intake: load the balls first.");
        telemetry.addLine(String.format("kF: open loop, power %.1f to %.1f by %.1f. Run a motor's kF first: its kP uses it.",
                POWER_START, POWER_END, POWER_INCREMENT));
        telemetry.addLine(String.format("Intake sequences run the other side open loop at %.1f power.", OTHER_INTAKE_POWER));
        for (Tuned t : all) {
            telemetry.addLine(String.format("%s: %s = kF, %s = kP (uses kF %.5f, %s)", t.name, t.kfButton, t.kpButton,
                    t.tuningKf(), t.kfComplete ? "measured" : "config, kF not run"));
        }
        telemetry.addLine("X: cancel the running sequence (hold briefly)");
        telemetry.addLine("At the end copy FINAL kF and BEST kP into the config named on the result line. Files: RobotLogs/");
    }

    /** One motor's kF and kP so far, and what its config has now. */
    private void addSummary(Tuned t) {
        String kf = t.kfSteps > 0
                ? String.format("%s kF %.5f (%d steps)", t.kfComplete ? "FINAL" : "so far", t.averageKf(), t.kfSteps)
                : "kF not measured";
        String kp;
        if (t.kpTuner == null) {
            kp = "kP not tuned";
        } else if (Double.isNaN(t.kpTuner.getBestKp())) {
            kp = "kP: " + t.kpTuner.getStatus();
        } else {
            kp = String.format("BEST kP %.5f %s (with kF %.5f, average of RPS bests %.5f)", t.kpTuner.getBestKp(),
                    t.kpTuner.isVerified() ? "VERIFIED" : "not verified: " + t.kpTuner.getStatus(), t.kpTuner.getKf(),
                    t.kpTuner.getAverageKp());
        }
        telemetry.addData(t.name, "%s | %s | %s now kF %.5f kP %.5f", kf, kp, t.configNames, t.configKf(), t.configKp());
    }

    /** One motor's kF per step and kP per RPS, shown before each telemetry.update() so they stay on screen. */
    private void addDetails(Tuned t) {
        telemetry.addLine("--- " + t.name + " kF per step ---");
        for (String line : t.stepKfLines) {
            telemetry.addLine(line);
        }
        if (t.kpTuner != null) {
            telemetry.addLine("--- " + t.name + " kP per RPS (kP, settle time; OSC = oscillated) ---");
            for (KpTuner.Target target : t.kpTuner.getTargets()) {
                telemetry.addLine(target.bestText());
                telemetry.addLine(target.trialsText());
            }
            if (!t.kpTuner.getVerify().isEmpty()) {
                telemetry.addLine("--- " + t.name + " BEST kP at each RPS (verify) ---");
                for (KpTuner.Trial trial : t.kpTuner.getVerify()) {
                    telemetry.addLine(String.format("%.1f RPS: kP %s  overshoot %.1f  error %.1f", trial.targetRps,
                            trial.shortText(), trial.overshootRps, trial.errorRps));
                }
            }
        }
    }

    private static void flush(KFileWriter writer) {
        if (writer == null) {
            return;
        }
        try {
            writer.flush();
        } catch (java.io.IOException e) {
            KLog.e(TAG, "event=flushFailed", e);
        }
    }

    /**
     * Initialize the shooter motor with proper configuration
     */
    private KMotor initializeShooter() {
        DcMotor motor = hardwareMap.dcMotor.get("shooter");
        motor.setDirection(ShooterConfig.SHOOTER_MOTOR_DIRECTION); // same constant Shooter uses
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        KLog.d(TAG, "event=motorsInit");
        return new KMotor(motor,
            ShooterConfig.kp, ShooterConfig.ki, ShooterConfig.kd, ShooterConfig.kf);
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
