package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;

import com.acmerobotics.dashboard.config.Config;

/**
 * Tuning for the analog beam break sensors that count balls held in the robot.
 *
 * Fields are intentionally non-final so they can be tuned live from the FTC Dashboard while
 * BeamBreakTest is running - that OpMode prints the observed voltage window per sensor, which is
 * what you set the thresholds from.
 */
@Config
public class BeamBreakConfig {

    //========POLARITY========
    /**
     * True when an interrupted beam pulls the analog output LOW (the common case: output is near
     * the supply rail while the beam is intact, and collapses toward 0V when a ball blocks it).
     * Flip to false if your sensors read high when blocked.
     */
    public static boolean BLOCKED_IS_LOW_VOLTAGE = true;

    //========THRESHOLDS========
    /**
     * Per-sensor switching voltage, in volts, in BeamBreakPosition order
     * (A_OUTER, A_INNER, B_INNER, B_OUTER). Set each one to roughly the midpoint between the
     * blocked and clear voltages that BeamBreakTest reports for that sensor.
     */
    public static double THRESHOLD_V_A_OUTER = 1.6;
    public static double THRESHOLD_V_A_INNER = 1.6;
    public static double THRESHOLD_V_B_INNER = 1.6;
    public static double THRESHOLD_V_B_OUTER = 1.6;

    /**
     * Total width of the Schmitt-trigger band around the threshold, in volts. The raw reading has
     * to swing HYSTERESIS_V/2 past the threshold to flip, so a sensor sitting right at its
     * threshold cannot chatter. Widen this if the raw state oscillates with the ball at rest;
     * narrow it if a genuinely blocked beam never registers.
     */
    public static double HYSTERESIS_V = 0.25;

    //========DEBOUNCE========
    /**
     * Asymmetric debounce. A reading has to hold for this long before the filtered state follows
     * it, and the two directions are deliberately very different:
     *
     * CLEAR -> BLOCKED is short, because a ball arriving is real and we want the count to react.
     * BLOCKED -> CLEAR is long, because the balls are pickleball-style with holes in them: a hole
     * lining up with the beam makes a seated ball read clear for a few milliseconds. A clear
     * debounce longer than that glitch rides straight through it without the count twitching.
     *
     * Raise CLEAR_DEBOUNCE_MS if the count still dips while balls sit in the robot. Keep it below
     * the time it takes a ball to actually leave its position, or exits register late.
     */
    public static double BLOCKED_DEBOUNCE_MS = 20;
    public static double CLEAR_DEBOUNCE_MS = 150;

    //========PATTERN CORRECTION========
    /**
     * How long every sensor has to hold its filtered state before the plausibility pass is
     * allowed to correct anything.
     *
     * The pass assumes the balls have finished settling. A ball still moving through the robot can
     * legitimately produce an "impossible" pattern for a moment - it is blocking an inner beam
     * while the outer one it has already passed reads clear - and correcting that would count a
     * ball that is not there. A hole-induced false clear, by contrast, lasts as long as the ball
     * sits in that orientation, so it survives this gate and still gets corrected.
     *
     * The cost is that a real correction takes this much longer to appear. Lower it if the count
     * is slow to recover from a hole glitch; raise it if the count overshoots while intaking.
     */
    public static double CORRECTION_SETTLE_MS = 250;

    /**
     * Enables the physical-plausibility pass in BeamBreakArray. The intake is double sided and
     * balls stack from the outside in, so the occupied positions always form a block at the A end
     * plus a block at the B end, which means the EMPTY positions are always one contiguous run in
     * the middle. Any other pattern is physically impossible and means a sensor is falsely
     * reading clear through a ball's hole, so the array trusts the longest run of clear readings
     * and counts the stragglers as blocked.
     *
     * Turn this off to see the uncorrected sensor count (BeamBreakTest shows both either way).
     */
    public static boolean ENABLE_PATTERN_CORRECTION = true;

    //========FAULT DETECTION========
    /**
     * Voltage window a healthy sensor is expected to stay inside. A disconnected analog input
     * floats or sits pinned at a rail, so a reading outside this window is reported as a fault by
     * BeamBreakSensor.isFaulted() rather than silently counted as a ball. Defaults are wide
     * enough to accept any real 3.3V sensor - tighten them once you know your own sensors' range.
     */
    public static double MIN_VALID_V = 0.02;
    public static double MAX_VALID_V = 3.28;
}
