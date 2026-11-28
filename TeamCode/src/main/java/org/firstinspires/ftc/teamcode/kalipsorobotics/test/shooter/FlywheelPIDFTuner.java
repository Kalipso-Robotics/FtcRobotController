package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.PID.PIDFController;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KMotor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * Hand tune the flywheel kF (velocity feedforward) and kP (proportional to the RPS error), one gamepad.
 * The flywheel runs the same KMotor.goToRPS loop TeleOp uses: power = (kF * target + kP * error) * DEFAULT_VOLTAGE / battery.
 * kF / kP edits (gamepad or Dashboard ShooterConfig) apply live. Nothing is saved: copy the final kF / kP into ShooterConfig.
 *
 * GP1  X  flywheel on/off            A  recovery test (drop DROP_RPS, then time the climb back to TARGET_RPS)
 *      Y  mode: kF ONLY (starts here) / kF + kP
 *      dpad up/down    kP +/- KP_STEP          dpad right/left  kF +/- KF_STEP
 *      RB / LB         target RPS +/- 5
 * Dashboard (FlywheelPIDFTuner): TARGET_RPS, DROP_RPS, KP_STEP, KF_STEP. Graph errorPct, errorAvgPct, rps.
 *
 * What runs in each mode (see gains()):
 *  kF ONLY  kP = kI = kD = 0 in the controller, whatever ShooterConfig says (ShooterConfig.kp is kept, not zeroed).
 *           dpad up/down and A are ignored. Pure feedforward, so errorAvgPct shows kF's error alone.
 *  kF + kP  ShooterConfig kp / ki / kd, the same gains TeleOp builds the Shooter with.
 *  Both     voltage compensation ON, as in TeleOp (ShooterConfig.VOLTAGE_COMP_EXPONENT): kF is power at 12 V per RPS,
 *           the same definition MotorPowerRPSDataCollector measures. Bang-bang (ShooterRun only) is never used here.
 *
 * Procedure
 *  1. kF, in kF ONLY mode. X on at TARGET_RPS. Wait ~2 s, then read errorAvgPct:
 *     positive = too slow, raise kF; negative = too fast, lower kF. Repeat at ~30, 40 and 50 RPS (RB / LB).
 *     Pick the kF that keeps errorAvgPct within about +/-2% at every RPS (kF alone always lands a bit off).
 *  2. kP. Y to kF + kP mode. Raise kP in KP_STEP steps. errorAvgPct shrinks toward 0. Keep raising until rps visibly hunts up and down
 *     on the Dashboard graph (errorPct swinging more than ~3% back and forth), then back kP off to about half of that.
 *  3. Recovery. Press A at the shooting RPS: the target drops DROP_RPS (about one ball's dip), holds, then jumps back up;
 *     the time to get back inside RECOVERY_TOL_PCT and stay there is recovery ms. Raise kP while recovery ms falls and
 *     overshoot stays small. If power shows 1.00 during the climb, the motor is maxed out: more kP will not help.
 *  4. Re-check step 1's errorAvgPct with the final kP, at all three RPS. Copy kF / kP into ShooterConfig.
 *
 * Good enough (estimates for this motor and hub; TODO-measure against real shots):
 *  errorAvgPct  |x| < 1% good, < 2% acceptable. The hub reports velocity in ~0.7 RPS steps (~1.8% at 40 RPS), so the
 *               instant errorPct jumps around; judge the 1 s average.
 *  recovery ms  for a 5 RPS drop: < 400 ms good, < 700 ms acceptable, more = shots too slow back to back.
 *  overshoot    < 3% good. More means kP is too high (the next ball leaves fast).
 */
@Config
@TeleOp(name = "FlywheelPIDFTuner", group = "Test")
public class FlywheelPIDFTuner extends KOpMode {
    private static final String TAG = "Test_FlywheelPIDF";

    public static double TARGET_RPS = 40;
    public static double DROP_RPS = 5; // ~one ball's dip
    public static double KP_STEP = 0.0025;
    public static double KF_STEP = 0.0002;
    /** Recovered = inside this % of the target (never tighter than one hub velocity step) for HOLD_MS. */
    public static double RECOVERY_TOL_PCT = 2;
    static final double ENCODER_STEP_RPS = 0.75;
    static final double HOLD_MS = 300;
    static final double TIMEOUT_MS = 3000;
    private static final long AVG_MS = 1000;
    private static final double TARGET_STEP_RPS = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Shooter shooter = new Shooter(opModeUtilities);
        PIDFController pidf = shooter.getShooterMotor().getPIDFController();
        ShooterDataCollector.SteadyWindow avg = new ShooterDataCollector.SteadyWindow(AVG_MS);
        RecoveryTest recovery = new RecoveryTest();
        boolean on = false;
        boolean kfOnly = true; // step 1 first: kP cannot run until Y is pressed
        double lastTarget = Double.NaN;

        telemetry.addLine("FlywheelPIDFTuner: procedure is in the class comment. X = on, Y = mode (starts kF ONLY)");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            long nowMs = System.currentTimeMillis();

            if (kGamePad1.isButtonXFirstPressed()) {
                on = !on;
                recovery.cancel();
                if (!on) {
                    shooter.stop();
                }
            }
            if (kGamePad1.isButtonYFirstPressed()) {
                kfOnly = !kfOnly;
                recovery.cancel();
                avg.clear(); // the average must not mix the two modes
            }
            if (kGamePad1.isButtonAFirstPressed() && on && !kfOnly) { // recovery tests kP
                recovery.start(nowMs);
            }
            if (kGamePad1.isRightBumperFirstPressed()) {
                TARGET_RPS += TARGET_STEP_RPS;
            }
            if (kGamePad1.isLeftBumperFirstPressed()) {
                TARGET_RPS = Math.max(DROP_RPS + TARGET_STEP_RPS, TARGET_RPS - TARGET_STEP_RPS);
            }
            int kpSteps = (kGamePad1.isDpadUpFirstPressed() ? 1 : 0) - (kGamePad1.isDpadDownFirstPressed() ? 1 : 0);
            if (!kfOnly) {
                ShooterConfig.kp = Math.max(0, ShooterConfig.kp + KP_STEP * kpSteps);
            }
            ShooterConfig.kf = Math.max(0, ShooterConfig.kf
                    + KF_STEP * ((kGamePad1.isDpadRightFirstPressed() ? 1 : 0) - (kGamePad1.isDpadLeftFirstPressed() ? 1 : 0)));
            double[] pid = gains(kfOnly, ShooterConfig.kp, ShooterConfig.ki, ShooterConfig.kd);
            pidf.setKp(pid[0]); // every loop: Dashboard edits apply live too
            pidf.setKi(pid[1]);
            pidf.setKd(pid[2]);
            pidf.setKf(ShooterConfig.kf);

            double target = recovery.target(TARGET_RPS, DROP_RPS);
            if (target != lastTarget) {
                avg.clear(); // average only this target
                lastTarget = target;
            }
            if (on) {
                shooter.goToRPS(target);
            }
            double rps = shooter.getRPS();
            avg.add(nowMs, rps, 0);
            recovery.update(nowMs, rps, TARGET_RPS, DROP_RPS);

            double errorPct = on ? 100 * (target - rps) / target : 0;
            double errorAvgPct = on ? 100 * (target - avg.rpsAvg()) / target : 0;
            telemetry.addData("mode (Y)", kfOnly ? "kF ONLY (kP/kI/kD forced 0)" : "kF + kP");
            telemetry.addData("on", on);
            telemetry.addData("targetRPS", "%.1f", on ? target : 0);
            telemetry.addData("rps", "%.2f", rps);
            telemetry.addData("errorPct", "%.2f", errorPct);
            telemetry.addData("errorAvgPct (1 s, goal |x| < 1)", "%.2f", errorAvgPct);
            telemetry.addData("recovery (A; goal < 400 ms, overshoot < 3%)", recovery.result);
            telemetry.addData("kP (active / saved)", "%.5f / %.5f", pid[0], ShooterConfig.kp);
            telemetry.addData("kI / kD (active)", "%.5f / %.5f", pid[1], pid[2]);
            telemetry.addData("kF", "%.5f", ShooterConfig.kf);
            telemetry.addData("power", "%.2f", shooter.getShooterMotor().getPower());
            telemetry.addData("voltage", "%.2f", SharedData.getVoltage());
            telemetry.addData("voltage comp (exponent / scale)", "%.2f / %.3f",
                    ShooterConfig.VOLTAGE_COMP_EXPONENT, KMotor.voltageScale(SharedData.getVoltage()));
            telemetry.update();

            updateActions(); // refreshes the battery voltage the power is compensated with
        }
        shooter.stop();
        KLog.d(TAG, () -> String.format("event=done kp=%.5f kf=%.5f (copy into ShooterConfig)", ShooterConfig.kp, ShooterConfig.kf));
        cleanupRobot();
    }

    /** {kP, kI, kD} the controller runs: all 0 in kF-only mode (pure feedforward), else the config gains. */
    static double[] gains(boolean kfOnly, double kp, double ki, double kd) {
        return kfOnly ? new double[] {0, 0, 0} : new double[] {kp, ki, kd};
    }

    /** Drop the target by DROP_RPS until it holds there, then time the climb back to the high target. */
    static class RecoveryTest {
        enum Phase { IDLE, DROP, RECOVER }

        Phase phase = Phase.IDLE;
        String result = "press A";
        private double startMs;
        private double insideSinceMs = Double.NaN;
        private double overshootPct;

        double target(double high, double drop) {
            return phase == Phase.DROP ? high - drop : high;
        }

        void start(double nowMs) {
            phase = Phase.DROP;
            startMs = nowMs;
            insideSinceMs = Double.NaN;
            result = "dropping";
        }

        void cancel() {
            if (phase != Phase.IDLE) {
                phase = Phase.IDLE;
                result = "cancelled";
            }
        }

        void update(double nowMs, double rps, double high, double drop) {
            if (phase == Phase.IDLE) {
                return;
            }
            double target = target(high, drop);
            boolean inside = Math.abs(target - rps) <= Math.max(RECOVERY_TOL_PCT / 100 * target, ENCODER_STEP_RPS);
            if (!inside) {
                insideSinceMs = Double.NaN;
            } else if (Double.isNaN(insideSinceMs)) {
                insideSinceMs = nowMs;
            }
            boolean held = !Double.isNaN(insideSinceMs) && nowMs - insideSinceMs >= HOLD_MS;
            boolean timedOut = nowMs - startMs > TIMEOUT_MS;

            if (phase == Phase.DROP) {
                if (held || timedOut) { // at the low RPS (or close enough): jump back up and start the clock
                    phase = Phase.RECOVER;
                    startMs = nowMs;
                    insideSinceMs = Double.NaN;
                    overshootPct = 0;
                    result = "recovering";
                }
                return;
            }
            overshootPct = Math.max(overshootPct, 100 * (rps - target) / target);
            if (held) {
                phase = Phase.IDLE;
                result = String.format("%.0f ms, overshoot %.1f%%", insideSinceMs - startMs, overshootPct);
                KLog.d(TAG, () -> "event=recovery " + result + String.format(" kp=%.5f kf=%.5f", ShooterConfig.kp, ShooterConfig.kf));
            } else if (timedOut) {
                phase = Phase.IDLE;
                result = "did not recover in 3 s (kF / kP too low?)";
            }
        }
    }
}
