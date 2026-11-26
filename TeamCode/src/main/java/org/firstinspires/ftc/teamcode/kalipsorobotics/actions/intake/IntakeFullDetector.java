package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig.BALL_1_MA;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig.BALL_2_MA;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig.BALL_3_MA;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig.BALL_4_MA;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig.STALL_CONFIRM_MS;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig.STALL_CURRENT_MA;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig.STALL_FILTER;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig.STALL_IGNORE_MS;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig.STALL_RPS;

/**
 * Full detection without beam breaks: a packed intake stalls against the closed stopper, so the running side's
 * current goes up and its RPS collapses. Full = filtered current >= STALL_CURRENT_MA AND RPS <= STALL_RPS for
 * STALL_CONFIRM_MS (same rule as DECODE's RunIntake). Thresholds live in IntakeConfig; tune with IntakeMotorTest.
 * Feed it the running side every loop; reset() when that side starts running.
 */
public class IntakeFullDetector {
    public static final int CAPACITY = 4;

    private long startMs;
    private double filteredMA = 0;
    private boolean primed = false;
    private long stalledSinceMs = -1;
    private boolean full = false;

    /** Call when the intake starts running. Also clears the full latch. */
    public void reset(long nowMs) {
        startMs = nowMs;
        filteredMA = 0;
        primed = false;
        stalledSinceMs = -1;
        full = false;
    }

    public void update(double currentMA, double rps, long nowMs) {
        if (nowMs - startMs < STALL_IGNORE_MS) {
            return;
        }
        filteredMA = primed ? filteredMA + STALL_FILTER * (currentMA - filteredMA) : currentMA;
        primed = true;
        boolean stalled = filteredMA >= STALL_CURRENT_MA && rps <= STALL_RPS;
        if (!stalled) {
            stalledSinceMs = -1;
        } else if (stalledSinceMs < 0) {
            stalledSinceMs = nowMs;
        }
        if (stalledSinceMs >= 0 && nowMs - stalledSinceMs >= STALL_CONFIRM_MS) {
            full = true; // latched: the current drops again once the motor stops, the balls are still in
        }
    }

    public boolean isFull() {
        return full;
    }

    /** Rough ball count: how many of IntakeConfig.BALL_1..4_MA the filtered current has reached. */
    public int estimateBalls() {
        if (full) {
            return CAPACITY;
        }
        if (!primed) {
            return 0;
        }
        int balls = 0;
        for (double threshold : new double[]{BALL_1_MA, BALL_2_MA, BALL_3_MA, BALL_4_MA}) {
            if (filteredMA >= threshold) {
                balls++;
            }
        }
        return balls;
    }

    /** The stall condition right now (before STALL_CONFIRM_MS), for tuning telemetry. */
    public boolean isStalling() {
        return stalledSinceMs >= 0;
    }

    /** How long the stall condition has held, 0 if it isn't. */
    public long stallHeldMs(long nowMs) {
        return stalledSinceMs < 0 ? 0 : nowMs - stalledSinceMs;
    }

    /** True while the spin-up window (STALL_IGNORE_MS) is still masking the readings. */
    public boolean isIgnoring(long nowMs) {
        return nowMs - startMs < STALL_IGNORE_MS;
    }

    public double getFilteredMA() {
        return filteredMA;
    }
}
