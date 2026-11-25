package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KMotor;

/**
 * Open-loop kF sweep for one or more motors (IntakeTest, ShooterDataCollector). Each step holds one power for stepMs;
 * after settleMs of spin-up, every loop takes kF = power / (RPS * KMotor.voltageScale(voltage)) per motor (skipped while
 * a motor is not moving forward). A step's kF is the average of its samples; a motor's result is the average of its
 * step kFs. The caller sets the motors to power() every loop and calls update().
 */
public class KfSweep {
    public static final double[] POWERS = {0.3, 0.5, 0.7, 0.9, 1.0}; // no 0.1: the motor stalls there and skews the average
    public static final double STEP_MS = 10000;
    public static final double SETTLE_MS = 2000; // spin-up after each power change: not counted in that step's kF
    private static final double MIN_RPS = 0.1;

    private final String tag;
    private final String[] names;
    private final double[] powers;
    private final double stepMs;
    private final double settleMs;
    private final ElapsedTime timer = new ElapsedTime();
    private final double[] sum;
    private final int[] count;
    private final double[][] stepKf;
    private final double[][] stepRps;
    private final double[] stepVoltage;
    private final double[] result;
    private final double[] kfNow; // this loop's sample, NaN while spinning up or not moving
    private int step = -1; // index into powers, -1 = not running
    private int doneSteps = 0;
    private String status = "not run";

    public KfSweep(String tag, String... names) {
        this(tag, POWERS, STEP_MS, SETTLE_MS, names);
    }

    /** Custom powers and timing (unit tests). */
    public KfSweep(String tag, double[] powers, double stepMs, double settleMs, String... names) {
        this.tag = tag;
        this.names = names;
        this.powers = powers;
        this.stepMs = stepMs;
        this.settleMs = settleMs;
        sum = new double[names.length];
        count = new int[names.length];
        stepKf = new double[names.length][powers.length];
        stepRps = new double[names.length][powers.length];
        stepVoltage = new double[powers.length];
        result = new double[names.length];
        kfNow = new double[names.length];
    }

    public void start() {
        step = 0;
        doneSteps = 0;
        timer.reset();
        java.util.Arrays.fill(sum, 0);
        java.util.Arrays.fill(count, 0);
        java.util.Arrays.fill(result, Double.NaN);
        status = "running";
        KLog.d(tag, "event=sweepStart");
    }

    public void cancel() {
        if (isRunning()) {
            step = -1;
            status = "cancelled";
            KLog.d(tag, "event=sweepCancel");
        }
    }

    public boolean isRunning() {
        return step >= 0;
    }

    /** The power to command this loop (0 when not running). */
    public double power() {
        return isRunning() ? powers[step] : 0;
    }

    /**
     * Call once per loop after the motors ran at power() since the last call. rps[i] is motor i's speed now.
     * Returns true on the loop the sweep finishes.
     */
    public boolean update(double[] rps, double voltage) {
        if (!isRunning()) {
            return false;
        }
        double p = powers[step];
        boolean measuring = timer.milliseconds() >= settleMs;
        for (int i = 0; i < names.length; i++) {
            kfNow[i] = measuring && rps[i] > MIN_RPS ? p / (rps[i] * KMotor.voltageScale(voltage)) : Double.NaN;
            if (!Double.isNaN(kfNow[i])) {
                sum[i] += kfNow[i];
                count[i]++;
            }
        }
        if (timer.milliseconds() < stepMs) {
            return false;
        }
        stepVoltage[step] = voltage;
        for (int i = 0; i < names.length; i++) {
            stepKf[i][step] = count[i] > 0 ? sum[i] / count[i] : Double.NaN;
            stepRps[i][step] = rps[i];
            sum[i] = 0;
            count[i] = 0;
            int st = step, m = i;
            KLog.d(tag, () -> String.format("event=sweepStep motor=%s power=%.2f rps=%.2f kF=%.5f voltage=%.2f",
                    names[m], powers[st], stepRps[m][st], stepKf[m][st], voltage));
        }
        doneSteps = ++step;
        timer.reset();
        if (step < powers.length) {
            return false;
        }
        step = -1;
        status = "done";
        for (int i = 0; i < names.length; i++) {
            double total = 0;
            int n = 0;
            for (double kf : stepKf[i]) {
                if (!Double.isNaN(kf)) {
                    total += kf;
                    n++;
                }
            }
            result[i] = n > 0 ? total / n : Double.NaN;
            int m = i, used = n;
            KLog.d(tag, () -> String.format("event=sweepDone motor=%s kF=%.5f steps=%d/%d", names[m], result[m], used, powers.length));
        }
        return true;
    }

    /** Average kF of motor i over its steps; NaN until a sweep finishes or if it never moved. */
    public double result(int i) {
        return result[i];
    }

    public int doneSteps() {
        return doneSteps;
    }

    public double stepPower(int k) {
        return powers[k];
    }

    public double stepKf(int i, int k) {
        return stepKf[i][k];
    }

    public double stepRps(int i, int k) {
        return stepRps[i][k];
    }

    /** Battery voltage at the end of step k. */
    public double stepVoltage(int k) {
        return stepVoltage[k];
    }

    public void addTelemetry(Telemetry telemetry, double[] rpsNow) {
        if (isRunning()) {
            telemetry.addData("SWEEP", "power %.1f (step %d/%d)  %.1f / %.0f s  %s", powers[step], step + 1, powers.length,
                    timer.milliseconds() / 1000, stepMs / 1000, timer.milliseconds() < settleMs ? "spinning up" : "measuring");
        }
        for (int i = 0; i < names.length; i++) {
            if (isRunning()) {
                telemetry.addData("sweep now " + names[i], "RPS %.2f  kF now %.5f  step kF so far %.5f",
                        rpsNow[i], kfNow[i], count[i] > 0 ? sum[i] / count[i] : Double.NaN);
            }
            StringBuilder steps = new StringBuilder();
            for (int k = 0; k < doneSteps; k++) {
                steps.append(String.format("%.1f: %.2f RPS kF %.5f  ", powers[k], stepRps[i][k], stepKf[i][k]));
            }
            telemetry.addData("sweep steps " + names[i], steps.length() > 0 ? steps.toString() : "-");
            String res = !"done".equals(status) ? status
                    : Double.isNaN(result[i]) ? "no kF: never moved forward" : String.format("avg kF %.5f", result[i]);
            telemetry.addData("SWEEP RESULT " + names[i], res);
        }
    }
}
