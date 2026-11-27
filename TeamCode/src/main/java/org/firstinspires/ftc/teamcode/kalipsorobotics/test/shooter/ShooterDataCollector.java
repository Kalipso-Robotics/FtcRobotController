package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.PID.PIDFController;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterReady;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher.PushBall;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.test.KfSweep;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KMotor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import java.util.ArrayDeque;

/**
 * Tunes the flywheel PF (no I or D) in three stages, judged by RPS % error = (target - RPS) / target.
 * Pattern source: the earlier ShooterDataCollector; the shot gate is ShootAllAction's (ShooterReady, then PushBall).
 * The drivetrain and turret are not used: the turret does not aim.
 *
 *   1 KF     Open loop sweep (KfSweep, the same one IntakeTest uses): RB runs the flywheel at 0.3, 0.5 ... 0.9, then 1.0,
 *            10 s each. After 2 s of spin-up each loop takes kF = power / (RPS * voltageScale); a step's kF is the average
 *            of its samples and kF = the average of the steps. It goes into ShooterConfig.kf when the sweep finishes.
 *   2 KP     Forced oscillation. The normal ShooterRun loop with kP starting at 0 (so the % error is kF's alone).
 *            Raise kP until the RPS keeps oscillating (OSCILLATING; kick it with an RPS step or a ball), then RB:
 *            that kP is Ku and kP becomes KU_FRACTION * Ku.
 *   3 SHOTS  ShooterRun + ShooterReady + PushBall. % error at the feed, drop % and recovery time. Trim kP here.
 *
 * kP and kF go into ShooterConfig live and stay until the app restarts (so TeleOp can try them): copy them into
 * ShooterConfig by hand. The kP being tried in stage 2 is not kept: leaving the stage or stopping goes back to the last accepted kP.
 *
 * GP1  X    flywheel on/off        Y    next stage (stops the flywheel)     LT   stop, cancel a running shot
 *      dpad up/down, right/left    power +/- POWER_STEP_BIG, _SMALL (stage 1) or RPS +/- RPS_STEP_BIG, _SMALL
 *      RB   start / cancel the kF sweep (1) / accept this kP as Ku (2)      LB   feed once ShooterReady (2, 3)
 *      X / LT / Y also cancel a running kF sweep.
 * GP2  dpad up/down, right/left    kP +/- KP_STEP_BIG, _SMALL (2, 3)        RT   intake (hold)
 *
 * KLog tag Test_Shooter: event=tune every TUNE_LOG_MS, plus stage, spin, rpsSet, powerSet, kpSet, sweep*, kuAccepted,
 * oscillation, shotArm, feed, shotDone, shotAborted, stepStart/stepDone, dipStart/dipDone.
 * Each sweep step also goes to RobotLogs/ShooterKfSamples_*.csv (two batteries at one power give ShooterConfig.VOLTAGE_COMP_EXPONENT).
 */
@Config
@TeleOp(name = "ShooterDataCollector", group = "Test")
public class ShooterDataCollector extends KOpMode {
    public static final String TAG = "Test_Shooter";

    public static double targetRPS = 40;
    public static double hoodPosition = 0.2;
    /** kP = this * Ku. 0.5 is the Ziegler-Nichols P value; lower is calmer. */
    public static double KU_FRACTION = 0.5;
    public static double SETTLE_TOLERANCE_PCT = 2.5; // TODO-measure
    /** RPS this far under a settled target counts as a shot dip. */
    public static double DIP_PCT = 6; // TODO-measure
    /** Must stay inside the tolerance this long to count as settled. */
    public static double SETTLE_HOLD_MS = 300;
    /** STEADY when the average RPS of the two halves of the window differs less than this. */
    public static double STEADY_DRIFT_PCT = 0.5; // TODO-measure
    /** RPS must swing this far to each side of its average to count as an oscillation. */
    public static double OSCILLATION_PCT = 2.5; // TODO-measure
    public static int OSCILLATION_MIN_CROSSINGS = 5;

    // Hub velocity comes in steps of 20 ticks/s = 0.71 RPS: no RPS band is tighter than one step.
    private static final double ENCODER_STEP_RPS = 0.75;
    private static final double RPS_STEP_BIG = 5;
    private static final double RPS_STEP_SMALL = 0.5;
    private static final double POWER_STEP_BIG = 0.05;
    private static final double POWER_STEP_SMALL = 0.01;
    private static final double KP_STEP_BIG = 0.01;
    private static final double KP_STEP_SMALL = 0.001;
    private static final long WINDOW_MS = 3000;
    private static final long TUNE_LOG_MS = 100;

    private enum Stage {
        KF("1/3 kF: RB = kF sweep (0.3 to 1.0 power, 10 s each, ~50 s), wait for SWEEP RESULT, then Y"),
        KP("2/3 kP: X on, raise kP (GP2 dpad) until OSCILLATING, RB = accept as Ku, then Y"),
        SHOTS("3/3 shots: X on, GP2 RT = load balls, LB = ShooterReady + PushBall. Trim kP (GP2 dpad)");

        final String prompt;

        Stage(String prompt) {
            this.prompt = prompt;
        }
    }

    private Stopper stopper;
    private Intake intake;
    private KActionSet shot;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // Dashboard graphs

        // No flower here: this OpMode only runs the intake motors
        intake = new Intake(new IntakeModule(opModeUtilities, IntakeSide.FRONT, IntakeConfig.SIDE_A),
                new IntakeModule(opModeUtilities, IntakeSide.BACK, IntakeConfig.SIDE_B), null);
        stopper = new Stopper(opModeUtilities);
        Shooter shooter = new Shooter(opModeUtilities);
        PIDFController pidf = shooter.getShooterMotor().getPIDFController();
        // updateActions() ticks shooterRun: it is this while the RPS loop owns the flywheel, else null.
        ShooterRun rpsLoop = new ShooterRun(opModeUtilities, shooter, targetRPS, hoodPosition);
        KFileWriter kfSamples = new KFileWriter("ShooterKfSamples", opModeUtilities);
        kfSamples.writeLine("power,rps,voltage,voltageCompExponent,kf");
        flush(kfSamples);
        stopper.setPosition(ModuleConfig.STOPPER_SERVO_CLOSED_POS);
        intake.lower();

        Stage stage = Stage.KF;
        boolean spinning = false;
        double shooterPower = 0.5;
        double acceptedKp = ShooterConfig.kp;
        double ku = 0;
        KfSweep sweep = new KfSweep(TAG, "flywheel");
        boolean kfMeasured = false;
        SteadyWindow window = new SteadyWindow(WINDOW_MS);
        ResponseTracker response = new ResponseTracker();
        ShooterReady shooterReady = null;
        double shotRps = 0;
        long shotStartMs = 0;
        boolean fed = false;
        boolean wasOscillating = false;
        String lastFeed = "none yet";
        String message = "";
        long lastTuneLogMs = 0;
        KLog.d(TAG, () -> String.format("event=start kp=%.5f kf=%.5f voltageCompExponent=%.2f defaultVoltage=%.2f bangBang=%b",
                ShooterConfig.kp, ShooterConfig.kf, ShooterConfig.VOLTAGE_COMP_EXPONENT,
                ShooterInterpolationConfig.DEFAULT_VOLTAGE, ShooterConfig.shouldBangBangCompensate));
        telemetry.addLine(stage.prompt);
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            intake.update();
            if (!isPending(shot)) { // PushBall owns the intake while a shot runs
                if (kGamePad2.isRightTriggerPressed()) {
                    intake.run(1);
                } else {
                    intake.stop();
                }
            }

            // ---- Flywheel on/off and stage ----
            boolean stopPressed = kGamePad1.isLeftTriggerFirstPressed();
            boolean nextPressed = kGamePad1.isButtonYFirstPressed();
            boolean togglePressed = kGamePad1.isButtonXFirstPressed();
            if (sweep.isRunning() && (stopPressed || nextPressed || togglePressed)) {
                sweep.cancel();
                shooter.stop();
                togglePressed = false; // this X only cancels the sweep, it does not also start the flywheel
                message = "kF sweep cancelled: kF unchanged";
            }
            if (stopPressed || nextPressed || (togglePressed && spinning)) {
                spinning = false;
                shooterRun = null;
                cancelShot();
                shooter.stop();
                shooter.setTargetRPS(0);
                window.clear();
                response.reset();
                KLog.d(TAG, "event=stopShooter");
            } else if (togglePressed) {
                spinning = true;
                window.clear();
                Stage s = stage;
                KLog.d(TAG, () -> "event=spin stage=" + s);
            }
            if (nextPressed) {
                message = stage == Stage.KF && !kfMeasured ? "No kF sweep finished: keeping the current kF" : "";
                if (stage == Stage.KP) {
                    ShooterConfig.kp = acceptedKp;
                }
                stage = Stage.values()[(stage.ordinal() + 1) % Stage.values().length];
                if (stage == Stage.KP) {
                    acceptedKp = ShooterConfig.kp;
                    ShooterConfig.kp = 0;
                    ku = 0;
                } else if (stage == Stage.KF) {
                    kfMeasured = false;
                }
                Stage s = stage;
                KLog.d(TAG, () -> String.format("event=stage stage=%s kp=%.5f kf=%.5f", s, ShooterConfig.kp, ShooterConfig.kf));
            }
            boolean closedLoop = stage != Stage.KF;

            // ---- Power / RPS setpoint ----
            double steps = (kGamePad1.isDpadUpFirstPressed() ? 1 : 0) - (kGamePad1.isDpadDownFirstPressed() ? 1 : 0);
            double fineSteps = (kGamePad1.isDpadRightFirstPressed() ? 1 : 0) - (kGamePad1.isDpadLeftFirstPressed() ? 1 : 0);
            if (steps != 0 || fineSteps != 0) {
                window.clear();
                if (closedLoop) {
                    targetRPS = MathFunctions.clamp(targetRPS + steps * RPS_STEP_BIG + fineSteps * RPS_STEP_SMALL,
                            0, ShooterInterpolationConfig.MAX_RPS);
                    KLog.d(TAG, () -> String.format("event=rpsSet targetRps=%.1f", targetRPS));
                } else {
                    shooterPower = MathFunctions.clamp(shooterPower + steps * POWER_STEP_BIG + fineSteps * POWER_STEP_SMALL, 0, 1);
                    double p = shooterPower;
                    KLog.d(TAG, () -> String.format("event=powerSet power=%.2f", p));
                }
            }

            // ---- kP, live ----
            double kpSteps = (kGamePad2.isDpadUpFirstPressed() ? 1 : 0) - (kGamePad2.isDpadDownFirstPressed() ? 1 : 0);
            double kpFineSteps = (kGamePad2.isDpadRightFirstPressed() ? 1 : 0) - (kGamePad2.isDpadLeftFirstPressed() ? 1 : 0);
            if (closedLoop && (kpSteps != 0 || kpFineSteps != 0)) {
                ShooterConfig.kp = Math.max(0, ShooterConfig.kp + kpSteps * KP_STEP_BIG + kpFineSteps * KP_STEP_SMALL);
                window.clear(); // judge the oscillation at this kP only
                KLog.d(TAG, () -> String.format("event=kpSet kp=%.5f", ShooterConfig.kp));
            }
            if (stage == Stage.SHOTS) {
                acceptedKp = ShooterConfig.kp;
            }
            pidf.setKp(ShooterConfig.kp); // every loop: Dashboard edits apply live too
            pidf.setKf(ShooterConfig.kf);

            // ---- Drive the flywheel ----
            if (spinning && closedLoop) {
                if (!isPending(shot)) { // a running shot keeps its own RPS
                    rpsLoop.setTargetRPS(targetRPS);
                    rpsLoop.setTargetHoodPosition(hoodPosition);
                }
                shooterRun = rpsLoop;
            } else if (sweep.isRunning()) {
                shooter.setPower(sweep.power());
            } else if (spinning) {
                shooter.setPower(shooterPower);
            }

            // ---- Measure ----
            long nowMs = System.currentTimeMillis();
            double rps = shooter.getRPS();
            double voltage = SharedData.getVoltage(); // the voltage KMotor compensates with
            double motorPower = shooter.getShooterMotor().getPower();
            double activeTarget = isPending(shot) ? shotRps : targetRPS;
            window.add(nowMs, rps, voltage);
            double rpsAvg = window.rpsAvg();
            boolean steady = spinning && window.isSteady(STEADY_DRIFT_PCT);
            if (sweep.update(new double[]{rps}, voltage)) { // RPS from the power set above
                shooter.stop();
                for (int k = 0; k < sweep.doneSteps(); k++) {
                    kfSamples.writeLine(String.format("%.2f,%.2f,%.2f,%.2f,%.5f", sweep.stepPower(k), sweep.stepRps(0, k),
                            sweep.stepVoltage(k), ShooterConfig.VOLTAGE_COMP_EXPONENT, sweep.stepKf(0, k)));
                }
                flush(kfSamples);
                if (Double.isNaN(sweep.result(0))) {
                    message = "kF sweep: the flywheel never moved, kF unchanged";
                } else {
                    ShooterConfig.kf = sweep.result(0);
                    kfMeasured = true;
                    message = String.format("kF sweep done: kF %.5f now in use (copy into ShooterConfig)", ShooterConfig.kf);
                }
            }
            double errorPct = closedLoop ? percent(activeTarget - rps, activeTarget) : 0;
            double errorAvgPct = closedLoop ? percent(activeTarget - rpsAvg, activeTarget) : errorPct;
            Oscillation oscillation = window.oscillation();
            boolean oscillating = spinning && closedLoop && !isPending(shot) && oscillation.sustained;
            if (oscillating != wasOscillating) {
                wasOscillating = oscillating;
                KLog.d(TAG, () -> String.format("event=oscillation on=%b %s kp=%.5f", oscillating, oscillation, ShooterConfig.kp));
            }
            if (spinning && closedLoop) {
                ResponseTracker.Event event = response.update(nowMs, activeTarget, rps);
                if (event != ResponseTracker.Event.NONE) {
                    KLog.d(TAG, () -> String.format("event=%s targetRps=%.1f rps=%.2f errorPct=%.2f %s kp=%.5f kf=%.5f voltage=%.2f",
                            event, activeTarget, rps, errorPct, response.summary(), ShooterConfig.kp, ShooterConfig.kf, voltage));
                }
            }
            if ((spinning || sweep.isRunning()) && nowMs - lastTuneLogMs >= TUNE_LOG_MS) {
                lastTuneLogMs = nowMs;
                Stage s = stage;
                boolean shooting = isPending(shot);
                KLog.d(TAG, () -> String.format("event=tune stage=%s targetRps=%.1f rps=%.2f rpsAvg=%.2f errorPct=%.2f errorAvgPct=%.2f spread=%.2f "
                                + "power=%.3f pTerm=%.4f fTerm=%.4f kp=%.5f kf=%.5f steady=%b %s voltage=%.2f voltageScale=%.3f shooting=%b",
                        s, closedLoop ? activeTarget : 0, rps, rpsAvg, errorPct, errorAvgPct, window.spread(), motorPower,
                        closedLoop ? ShooterConfig.kp * (activeTarget - rps) : 0, closedLoop ? ShooterConfig.kf * activeTarget : 0,
                        ShooterConfig.kp, ShooterConfig.kf, steady, oscillation, voltage, KMotor.voltageScale(voltage), shooting));
            }

            // ---- RB: kF sample / accept Ku ----
            if (kGamePad1.isRightBumperFirstPressed()) {
                if (stage == Stage.KF) {
                    if (sweep.isRunning()) {
                        sweep.cancel();
                        shooter.stop();
                        message = "kF sweep cancelled: kF unchanged";
                    } else {
                        spinning = false; // the sweep owns the flywheel
                        window.clear();
                        sweep.start();
                        message = "kF sweep running (~50 s)";
                    }
                } else if (stage == Stage.KP) {
                    if (!spinning || ShooterConfig.kp <= 0) {
                        message = "Ku needs the flywheel on and kP above 0";
                        KLog.d(TAG, "event=kuRefused");
                    } else {
                        ku = ShooterConfig.kp; // the driver decides: OSCILLATING is only a hint
                        acceptedKp = KU_FRACTION * ku;
                        ShooterConfig.kp = acceptedKp;
                        window.clear();
                        message = String.format("Ku %.5f -> kP %.5f%s", ku, acceptedKp, oscillating ? "" : "  (no oscillation was detected)");
                        double acceptedKu = ku;
                        KLog.d(TAG, () -> String.format("event=kuAccepted ku=%.5f kp=%.5f detected=%b %s",
                                acceptedKu, ShooterConfig.kp, oscillating, oscillation));
                    }
                }
            }

            // ---- LB: the real shot, ShooterReady then PushBall ----
            if (kGamePad1.isLeftBumperFirstPressed() && !isPending(shot)) {
                if (!spinning || !closedLoop) {
                    message = "Shooting needs stage 2 or 3 with the flywheel on";
                    KLog.d(TAG, "event=shotRefused");
                } else {
                    message = "";
                    shotRps = targetRPS;
                    shotStartMs = nowMs;
                    fed = false;
                    rpsLoop.update(); // isWithinRange must be for this target before ShooterReady reads it
                    shooterReady = new ShooterReady(rpsLoop); // TeleOp's gate
                    shooterReady.setName("shooterReady");
                    PushBall pushBall = new PushBall(stopper, intake);
                    pushBall.setName("pushBall");
                    pushBall.setDependentActions(shooterReady);
                    shot = new KActionSet();
                    shot.setName("tuneShot");
                    shot.addAction(shooterReady, pushBall);
                    setLastShooterAction(shot);
                    KLog.d(TAG, () -> String.format("event=shotArm targetRps=%.1f rps=%.2f errorPct=%.2f", targetRPS, rps, errorPct));
                }
            }

            telemetry.addLine("STEP " + stage.prompt);
            telemetry.addLine("GP1: X = on/off   Y = next step   LT = stop   dpad up/down big, right/left small");
            if (!message.isEmpty()) {
                telemetry.addLine(message);
            }
            telemetry.addLine("--- Flywheel ---");
            telemetry.addData("on", spinning || sweep.isRunning());
            if (closedLoop) {
                telemetry.addData("targetRPS", "%.1f", activeTarget);
            } else {
                telemetry.addData("set power", "%.2f", sweep.isRunning() ? sweep.power() : shooterPower);
            }
            telemetry.addData("actualRPS", "%.2f", rps);
            telemetry.addData("errorPct", "%.2f", errorPct);
            telemetry.addData("errorAvgPct", "%.2f", errorAvgPct);
            telemetry.addData("rps avg / spread (3 s)", "%.2f / %.2f  %s", rpsAvg, window.spread(), steady ? "STEADY" : "");
            telemetry.addData("motor power", "%.3f", motorPower);
            telemetry.addData("voltage", "%.2f", voltage);
            telemetry.addData("voltage comp exponent / scale", "%.2f / %.3f", ShooterConfig.VOLTAGE_COMP_EXPONENT, KMotor.voltageScale(voltage));
            telemetry.addLine("--- Tuning (copy to ShooterConfig) ---");
            telemetry.addData("kF in use", "%.5f", ShooterConfig.kf);
            sweep.addTelemetry(telemetry, new double[]{rps}); // live, per-step and final sweep kF, every stage
            telemetry.addData("kP", "%.5f", ShooterConfig.kp);
            if (stage != Stage.KF) {
                telemetry.addData("P term / F term", "%.4f / %.4f", ShooterConfig.kp * (activeTarget - rps), ShooterConfig.kf * activeTarget);
                telemetry.addData("oscillation", "%s %s", oscillating ? "OSCILLATING" : "no", oscillation);
                telemetry.addData("Ku / accepted kP", "%.5f / %.5f", ku, acceptedKp);
                telemetry.addData("response", response.summary());
                telemetry.addData("last RPS change", response.lastStep);
                telemetry.addData("last feed", lastFeed);
                telemetry.addData("last shot dip", response.lastDip);
                telemetry.addData("shot", !isPending(shot) ? "idle" : fed ? "PushBall" : "waiting for ShooterReady");
                if (ShooterConfig.shouldBangBangCompensate) {
                    telemetry.addLine("WARNING: bang-bang is on, kP only acts near the target");
                }
            }
            telemetry.addData("ballCount", intake.getBallCount());
            telemetry.update();

            updateActions();

            // ---- Shot progress ----
            if (shot != null) {
                long elapsedMs = System.currentTimeMillis() - shotStartMs;
                if (!fed && shooterReady.getIsDone()) {
                    fed = true;
                    lastFeed = String.format("ready=%dms errorPct=%.2f", elapsedMs, errorPct);
                    String f = lastFeed;
                    KLog.d(TAG, () -> String.format("event=feed targetRps=%.1f rps=%.2f %s voltage=%.2f", activeTarget, rps, f, voltage));
                } else if (!fed && shooterReady.timedOut()) { // TeleOp would not feed either
                    cancelShot();
                    message = String.format("Not fed: ShooterReady timed out at %.1f%% error", errorPct);
                    KLog.d(TAG, () -> String.format("event=shotAborted reason=notReady targetRps=%.1f rps=%.2f errorPct=%.2f", activeTarget, rps, errorPct));
                } else if (shot.getIsDone()) {
                    shot = null;
                    KLog.d(TAG, () -> "event=shotDone totalMs=" + elapsedMs);
                }
            }
        }

        if (stage == Stage.KP) {
            ShooterConfig.kp = acceptedKp; // static: don't leak the kP being tried into TeleOp
        }
        cancelShot();
        shooter.stop();
        intake.stop();
        kfSamples.close();
        KLog.d(TAG, () -> String.format("event=done kp=%.5f kf=%.5f", ShooterConfig.kp, ShooterConfig.kf));
        cleanupRobot();
    }

    private void cancelShot() {
        setLastShooterAction(null);
        if (isPending(shot)) {
            stopper.setPosition(ModuleConfig.STOPPER_SERVO_CLOSED_POS);
            intake.stop();
        }
        shot = null;
    }

    private static double percent(double part, double whole) {
        return whole == 0 ? 0 : 100 * part / whole;
    }

    /** pct of rps, but never tighter than one encoder step. */
    private static double band(double pct, double rps) {
        return Math.max(ENCODER_STEP_RPS, pct / 100 * Math.abs(rps));
    }

    /** Write the row to the file now, so a crash or a stop does not lose it. */
    private static void flush(KFileWriter rows) {
        try {
            rows.flush();
        } catch (java.io.IOException e) {
            KLog.e(TAG, "event=flushFailed", e);
        }
    }

    static class Oscillation {
        final boolean sustained;
        final double amplitudePct; // half of peak to peak, of the average RPS
        final double periodMs;     // 0 = fewer than 2 crossings

        Oscillation(boolean sustained, double amplitudePct, double periodMs) {
            this.sustained = sustained;
            this.amplitudePct = amplitudePct;
            this.periodMs = periodMs;
        }

        @Override
        public String toString() {
            return String.format("amplitudePct=%.2f periodMs=%.0f", amplitudePct, periodMs);
        }
    }

    /** The last windowMs of RPS and voltage. */
    static class SteadyWindow {
        private final long windowMs;
        private final ArrayDeque<double[]> samples = new ArrayDeque<>(); // ms, rps, voltage
        private double firstMs = Double.NaN;

        SteadyWindow(long windowMs) {
            this.windowMs = windowMs;
        }

        void clear() {
            samples.clear();
        }

        void add(double nowMs, double rps, double voltage) {
            if (samples.isEmpty()) {
                firstMs = nowMs;
            }
            samples.addLast(new double[]{nowMs, rps, voltage});
            while (samples.peekFirst()[0] < nowMs - windowMs) {
                samples.removeFirst();
            }
        }

        private boolean isFull() {
            return !samples.isEmpty() && samples.peekLast()[0] - firstMs >= windowMs;
        }

        /** Window is full and the RPS is not drifting: the averages of its two halves match. */
        boolean isSteady(double maxDriftPct) {
            if (!isFull()) {
                return false;
            }
            double oldAvg = rpsIn(true).average().orElse(Double.NaN); // NaN fails the check
            double newAvg = rpsIn(false).average().orElse(Double.NaN);
            return Math.abs(oldAvg - newAvg) < maxDriftPct / 100 * rpsAvg();
        }

        /** Sustained = enough swings through the average, and the newer half swings about as far as the older (not dying out). */
        Oscillation oscillation() {
            double average = rpsAvg();
            double band = band(OSCILLATION_PCT, average);
            int side = 0;
            int crossings = 0;
            double firstCrossMs = 0;
            double lastCrossMs = 0;
            for (double[] sample : samples) {
                int newSide = sample[1] > average + band ? 1 : sample[1] < average - band ? -1 : side;
                if (side != 0 && newSide != side) {
                    if (crossings == 0) {
                        firstCrossMs = sample[0];
                    }
                    lastCrossMs = sample[0];
                    crossings++;
                }
                side = newSide;
            }
            boolean sustained = isFull() && crossings >= OSCILLATION_MIN_CROSSINGS && swing(false) >= 0.7 * swing(true);
            return new Oscillation(sustained, percent(spread() / 2, average),
                    crossings < 2 ? 0 : 2 * (lastCrossMs - firstCrossMs) / (crossings - 1));
        }

        double rpsAvg() {
            return average(1);
        }

        double rpsMin() {
            return samples.stream().mapToDouble(sample -> sample[1]).min().orElse(0);
        }

        double rpsMax() {
            return samples.stream().mapToDouble(sample -> sample[1]).max().orElse(0);
        }

        double spread() {
            return rpsMax() - rpsMin();
        }

        private double swing(boolean olderHalf) {
            return rpsIn(olderHalf).max().orElse(0) - rpsIn(olderHalf).min().orElse(0);
        }

        private java.util.stream.DoubleStream rpsIn(boolean olderHalf) {
            double midMs = samples.peekLast()[0] - windowMs / 2.0;
            return samples.stream().filter(sample -> (sample[0] < midMs) == olderHalf).mapToDouble(sample -> sample[1]);
        }

        private double average(int column) {
            return samples.stream().mapToDouble(sample -> sample[column]).average().orElse(0);
        }
    }

    /** Times how the flywheel answers a target change (step) or a ball (dip). */
    static class ResponseTracker {
        enum Event { NONE, stepStart, stepDone, dipStart, dipDone }

        String lastStep = "none yet";
        String lastDip = "none yet";
        private double target = Double.NaN;
        private boolean active;
        private boolean dip;
        private double direction; // +1 spinning up, -1 spinning down
        private double startMs;
        private double nowMs;
        private double reachedMs;      // first time at the target, NaN before
        private double insideSinceMs;  // inside the tolerance since, NaN when outside
        private double overshoot;
        private double maxDrop;

        void reset() {
            target = Double.NaN;
            active = false;
        }

        Event update(double nowMs, double target, double rps) {
            this.nowMs = nowMs;
            if (target != this.target) { // also the first call: NaN is never equal
                this.target = target;
                begin(false, rps);
                return Event.stepStart;
            }
            if (!active) {
                if (target - rps > band(DIP_PCT, target)) {
                    begin(true, rps);
                    return Event.dipStart;
                }
                return Event.NONE;
            }
            double tolerance = band(SETTLE_TOLERANCE_PCT, target);
            maxDrop = Math.max(maxDrop, target - rps);
            if (Double.isNaN(reachedMs) && direction * (rps - target) >= -tolerance) {
                reachedMs = nowMs;
            }
            if (!Double.isNaN(reachedMs)) {
                overshoot = Math.max(overshoot, direction * (rps - target));
            }
            if (Math.abs(target - rps) > tolerance) {
                insideSinceMs = Double.NaN;
            } else if (Double.isNaN(insideSinceMs)) {
                insideSinceMs = nowMs;
            }
            if (Double.isNaN(insideSinceMs) || nowMs - insideSinceMs < SETTLE_HOLD_MS) {
                return Event.NONE;
            }
            active = false;
            if (dip) {
                lastDip = summary();
                return Event.dipDone;
            }
            lastStep = summary();
            return Event.stepDone;
        }

        /** Dip: how far it fell and how long until back. Step: rise time, settle time, overshoot. */
        String summary() {
            if (Double.isNaN(target)) {
                return "off";
            }
            String settle = active ? String.format("settling(%.0fms)", nowMs - startMs) : String.format("%.0fms", insideSinceMs - startMs);
            return dip
                    ? String.format("dropPct=%.1f recover=%s overshootPct=%.1f", percent(maxDrop, target), settle, percent(overshoot, target))
                    : String.format("rise=%.0fms settle=%s overshootPct=%.1f", reachedMs - startMs, settle, percent(overshoot, target));
        }

        private void begin(boolean dip, double rps) {
            this.dip = dip;
            active = true;
            startMs = nowMs;
            direction = rps <= target ? 1 : -1;
            reachedMs = Double.NaN;
            insideSinceMs = Double.NaN;
            overshoot = 0;
            maxDrop = dip ? target - rps : 0;
        }
    }
}
