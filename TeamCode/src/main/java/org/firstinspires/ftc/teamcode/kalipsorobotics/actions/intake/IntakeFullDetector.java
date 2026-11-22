package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import com.acmerobotics.dashboard.config.Config;

/**
 * V1 ball counting without beam breaks: the intake motor draws more current the more balls it is pushing against,
 * and a packed intake stalls. Feed it the running side's current every loop.
 *
 * Tune with IntakeMotorTest (it graphs the current on the Dashboard): note the current running empty, then with
 * 4 balls packed, and set EMPTY_CURRENT_MA and FULL_CURRENT_MA from those.
 */
@Config
public class IntakeFullDetector {
    /** Running with no balls, mA. */
    public static double EMPTY_CURRENT_MA = 1500; // TODO-measure with IntakeMotorTest
    /** Packed with 4 balls (stalling), mA. Full once the filtered current stays at or above this for FULL_HOLD_MS. */
    public static double FULL_CURRENT_MA = 4000; // TODO-measure with IntakeMotorTest
    /** How long the current must stay high before it counts as full, so one ball bumping through is not "full". */
    public static double FULL_HOLD_MS = 250; // TODO-measure
    /** Ignore the spin-up spike right after the motor starts. */
    public static double START_IGNORE_MS = 300; // TODO-measure
    /** Smoothing per update, 0..1 (higher = follows the raw current faster). */
    public static double FILTER = 0.3;
    public static int CAPACITY = 4;

    private long startMs;
    private double filteredMA = 0;
    private boolean primed = false;
    private long highSinceMs = -1;
    private boolean full = false;

    /** Call when the intake starts running. */
    public void reset(long nowMs) {
        startMs = nowMs;
        filteredMA = 0;
        primed = false;
        highSinceMs = -1;
        full = false;
    }

    public void update(double currentMA, long nowMs) {
        if (nowMs - startMs < START_IGNORE_MS) {
            return;
        }
        filteredMA = primed ? filteredMA + FILTER * (currentMA - filteredMA) : currentMA;
        primed = true;
        if (filteredMA < FULL_CURRENT_MA) {
            highSinceMs = -1;
        } else if (highSinceMs < 0) {
            highSinceMs = nowMs;
        }
        if (highSinceMs >= 0 && nowMs - highSinceMs >= FULL_HOLD_MS) {
            full = true; // latched: the current drops again once the robot stops pushing, the balls are still in
        }
    }

    public boolean isFull() {
        return full;
    }

    /** Rough ball count from where the current sits between empty and full. */
    public int estimateBalls() {
        if (full) {
            return CAPACITY;
        }
        if (!primed) {
            return 0;
        }
        // ponytail: linear between empty and full; if the graph shows steps per ball, swap in one threshold per ball
        double fraction = (filteredMA - EMPTY_CURRENT_MA) / (FULL_CURRENT_MA - EMPTY_CURRENT_MA);
        return (int) Math.max(0, Math.min(CAPACITY - 1, Math.round(fraction * CAPACITY)));
    }

    public double getFilteredMA() {
        return filteredMA;
    }
}
