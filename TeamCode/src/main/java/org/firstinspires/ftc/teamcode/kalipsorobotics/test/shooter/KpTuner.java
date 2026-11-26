package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds a velocity loop kP (MotorPowerRPSDataCollector: flywheel, intake A, intake B). At each target RPS it runs step-response trials with a
 * rising kP (x RATIO) until one oscillates, bisects between the last calm kP and the first oscillating one, then tries one
 * kP on each side of the fastest.
 * That RPS's best kP is the fastest settling calm trial; within TIE_MS of the fastest the lowest kP wins (calmer).
 *
 * The overall best is the average of the per-RPS bests, capped at the lowest kP limit (highest calm kP) of any RPS, then
 * verified: one more trial at every RPS. If one oscillates, kP is cut to VERIFY_CUT of it and all are verified again.
 *
 * Hardware-free: the caller runs the trials (Runner) and analyze() scores the recorded RPS.
 */
public class KpTuner {
    /** In tolerance = within one hub velocity step (20 ticks/s = 0.71 RPS) of the target. */
    public static final double TOLERANCE_RPS = 0.75;
    /** The RPS must swing this far to each side of its average to count as a swing. */
    public static final double SWING_RPS = 0.75;
    /** This many side changes after first reaching the target = oscillating. One overshoot is only 1-2. */
    public static final int OSCILLATION_SWINGS = 3;
    /** The RPS must stay in tolerance at least this long, up to the end of the trial, to count as settled. */
    public static final double SETTLE_HOLD_MS = 300;
    /** Average over the last this many ms: smooths the hub's velocity steps. Delays every settle time equally. */
    private static final double SMOOTH_MS = 100;
    private static final double TIE_MS = 50;
    private static final double RATIO = 1.6;
    private static final int BISECTIONS = 1;
    private static final double VERIFY_CUT = 0.8;
    private static final int MAX_VERIFY_ROUNDS = 5;
    // kP search range, as multiples of kF
    private static final double START_OVER_KF = 0.2;
    private static final double MIN_OVER_KF = 0.05;
    private static final double MAX_OVER_KF = 10;

    /** Runs one step response at this kP; null = cancelled. */
    public interface Runner {
        Trial run(double targetRps, double kp);
    }

    public static class Trial {
        public final double targetRps;
        public final double kp;
        public final double settleMs; // NaN = did not settle
        public final double overshootRps;
        public final double errorRps; // target - average RPS at the end
        public final boolean oscillating;

        Trial(double targetRps, double kp, double settleMs, double overshootRps, double errorRps, boolean oscillating) {
            this.targetRps = targetRps;
            this.kp = kp;
            this.settleMs = settleMs;
            this.overshootRps = overshootRps;
            this.errorRps = errorRps;
            this.oscillating = oscillating;
        }

        public String shortText() {
            return String.format("%.5f %s", kp, oscillating ? "OSC" : Double.isNaN(settleMs)
                    ? String.format("no settle (err %.1f)", errorRps) : String.format("%.2fs", settleMs / 1000));
        }
    }

    /** Everything tried at one RPS. */
    public static class Target {
        public final double rps;
        public final List<Trial> trials = new ArrayList<>();
        public Trial best = null; // null = every kP oscillated, or not finished
        public double limitKp = 0; // highest calm kP
        public boolean foundOscillation = false;

        Target(double rps) {
            this.rps = rps;
        }

        public String bestText() {
            return best == null ? String.format("%.0f RPS: no calm kP yet", rps)
                    : String.format("%.0f RPS: kP %s  overshoot %.1f  calm up to %.5f%s", rps, best.shortText(),
                            best.overshootRps, limitKp, foundOscillation ? "" : " (search limit)");
        }

        public String trialsText() {
            StringBuilder s = new StringBuilder("   tried: ");
            for (Trial t : trials) {
                s.append(t.shortText()).append(" | ");
            }
            return s.toString();
        }
    }

    private final String tag;
    private final double kf;
    private final double[] targetsRps;
    private final Runner runner;
    private final List<Target> targets = new ArrayList<>();
    private final List<Trial> verify = new ArrayList<>(); // last verify round
    private double bestKp = Double.NaN;
    private double averageKp = Double.NaN;
    private boolean verified = false;
    private String status = "not run";

    public KpTuner(String tag, double kf, double[] targetsRps, Runner runner) {
        this.tag = tag;
        this.kf = kf;
        this.targetsRps = targetsRps;
        this.runner = runner;
    }

    /** Blocking. False = cancelled (finished RPS values keep their results). */
    public boolean run() {
        double start = START_OVER_KF * kf;
        for (double rps : targetsRps) {
            status = String.format("tuning %.0f RPS", rps);
            Target target = new Target(rps);
            targets.add(target);
            if (!tune(target, start)) {
                status = "cancelled";
                return false;
            }
            if (target.best != null) {
                start = Math.max(START_OVER_KF * kf, target.best.kp / 3); // skip the far too low kPs at the next RPS
            }
            KLog.d(tag, () -> "event=kpTarget " + target.bestText());
        }
        double sum = 0, limit = Double.POSITIVE_INFINITY;
        int n = 0;
        for (Target t : targets) {
            if (t.best != null) {
                sum += t.best.kp;
                limit = Math.min(limit, t.limitKp);
                n++;
            }
        }
        if (n == 0) {
            status = "no calm kP found at any RPS";
            return true;
        }
        averageKp = sum / n;
        bestKp = Math.min(averageKp, limit);
        for (int round = 1; round <= MAX_VERIFY_ROUNDS; round++) {
            status = String.format("verifying kP %.5f (round %d)", bestKp, round);
            verify.clear();
            boolean calm = true;
            for (double rps : targetsRps) {
                Trial t = runner.run(rps, bestKp);
                if (t == null) {
                    status = "cancelled while verifying";
                    return false;
                }
                verify.add(t);
                log(t, "kpVerify");
                calm &= !t.oscillating;
            }
            if (calm) {
                verified = true;
                break;
            }
            if (round < MAX_VERIFY_ROUNDS) {
                bestKp *= VERIFY_CUT;
            }
        }
        status = verified ? "done" : "done, but the best kP still OSCILLATES at some RPS";
        KLog.d(tag, () -> String.format("event=kpBest kp=%.5f average=%.5f kf=%.5f verified=%b", bestKp, averageKp, kf, verified));
        return true;
    }

    private boolean tune(Target target, double startKp) {
        double lo = 0, hi = Double.POSITIVE_INFINITY, kp = startKp; // lo = highest calm, hi = lowest oscillating
        int bisections = 0;
        while (kp >= MIN_OVER_KF * kf && kp <= MAX_OVER_KF * kf) {
            Trial t = runner.run(target.rps, kp);
            if (t == null) {
                return false;
            }
            target.trials.add(t);
            log(t, "kpTrial");
            if (t.oscillating) {
                hi = Math.min(hi, kp);
            } else {
                lo = Math.max(lo, kp);
            }
            if (lo > 0 && hi < Double.POSITIVE_INFINITY) {
                if (bisections++ == BISECTIONS) {
                    break;
                }
                kp = Math.sqrt(lo * hi);
            } else {
                kp = t.oscillating ? kp / RATIO : kp * RATIO;
            }
        }
        Trial coarse = pickBest(target.trials, hi);
        if (coarse != null) {
            for (double refined : new double[]{coarse.kp / Math.sqrt(RATIO), coarse.kp * Math.sqrt(RATIO)}) {
                if (refined >= hi || refined > MAX_OVER_KF * kf) {
                    continue;
                }
                Trial t = runner.run(target.rps, refined);
                if (t == null) {
                    return false;
                }
                target.trials.add(t);
                log(t, "kpTrial");
                if (t.oscillating) {
                    hi = Math.min(hi, refined);
                }
            }
        }
        lo = 0; // a refined kP may have oscillated below the earlier limit
        for (Trial t : target.trials) {
            if (isCalm(t, hi)) {
                lo = Math.max(lo, t.kp);
            }
        }
        target.limitKp = lo;
        target.foundOscillation = hi < Double.POSITIVE_INFINITY;
        target.best = pickBest(target.trials, hi);
        return true;
    }

    /** Fastest calm trial below the lowest oscillating kP (lowest kP on a tie); none settled = the smallest final error. */
    static Trial pickBest(List<Trial> trials, double lowestOscillatingKp) {
        double fastest = Double.POSITIVE_INFINITY;
        for (Trial t : trials) {
            if (isCalm(t, lowestOscillatingKp) && t.settleMs < fastest) {
                fastest = t.settleMs;
            }
        }
        Trial best = null;
        for (Trial t : trials) {
            if (!isCalm(t, lowestOscillatingKp)) {
                continue;
            }
            if (fastest < Double.POSITIVE_INFINITY) {
                if (t.settleMs <= fastest + TIE_MS && (best == null || t.kp < best.kp)) {
                    best = t;
                }
            } else if (best == null || Math.abs(t.errorRps) < Math.abs(best.errorRps)) {
                best = t;
            }
        }
        return best;
    }

    private static boolean isCalm(Trial t, double lowestOscillatingKp) {
        return !t.oscillating && t.kp < lowestOscillatingKp;
    }

    /** Scores one step response; timesMs starts at 0 when the target was commanded. */
    public static Trial analyze(double targetRps, double kp, List<Double> timesMs, List<Double> rawRps) {
        int n = timesMs.size();
        double[] rps = new double[n];
        double windowSum = 0;
        int from = 0;
        for (int i = 0; i < n; i++) {
            windowSum += rawRps.get(i);
            while (timesMs.get(i) - timesMs.get(from) > SMOOTH_MS) {
                windowSum -= rawRps.get(from++);
            }
            rps[i] = windowSum / (i - from + 1);
        }
        int reach = n / 2; // never reached: look at the second half
        int lastOut = -1;
        boolean reached = false;
        double peak = 0;
        for (int i = 0; i < n; i++) {
            peak = Math.max(peak, rps[i]);
            if (!reached && rps[i] >= targetRps - TOLERANCE_RPS) {
                reached = true;
                reach = i;
            }
            if (Math.abs(rps[i] - targetRps) > TOLERANCE_RPS) {
                lastOut = i;
            }
        }
        double end = 0;
        int endFrom = (reach + n) / 2;
        for (int i = endFrom; i < n; i++) {
            end += rps[i];
        }
        end /= Math.max(1, n - endFrom);

        int swings = 0, side = 0;
        for (int i = reach; i < n; i++) {
            int now = rps[i] > end + SWING_RPS ? 1 : rps[i] < end - SWING_RPS ? -1 : 0;
            if (now != 0 && now != side) {
                if (side != 0) {
                    swings++;
                }
                side = now;
            }
        }
        boolean settled = n > 0 && lastOut < n - 1 && timesMs.get(n - 1) - timesMs.get(lastOut + 1) >= SETTLE_HOLD_MS;
        return new Trial(targetRps, kp, settled ? timesMs.get(lastOut + 1) : Double.NaN, Math.max(0, peak - targetRps),
                targetRps - end, swings >= OSCILLATION_SWINGS);
    }

    private void log(Trial t, String event) {
        KLog.d(tag, () -> String.format("event=%s targetRps=%.1f kp=%.5f settleMs=%.0f overshoot=%.2f error=%.2f oscillating=%b",
                event, t.targetRps, t.kp, t.settleMs, t.overshootRps, t.errorRps, t.oscillating));
    }

    /** NaN until found; check isVerified(). */
    public double getBestKp() {
        return bestKp;
    }

    public double getAverageKp() {
        return averageKp;
    }

    public double getKf() {
        return kf;
    }

    public boolean isVerified() {
        return verified;
    }

    public String getStatus() {
        return status;
    }

    public List<Target> getTargets() {
        return targets;
    }

    public List<Trial> getVerify() {
        return verify;
    }
}
