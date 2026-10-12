package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.BeamBreakConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

/**
 * One analog beam break sensor, with the two filtering stages that turn a noisy voltage into a
 * state you can count on.
 *
 * Stage 1, hysteresis: the voltage has to swing HYSTERESIS_V/2 past the sensor's threshold before
 * the raw state flips, so a beam sitting right at the edge cannot chatter.
 *
 * Stage 2, asymmetric debounce: the raw state has to hold for BLOCKED_DEBOUNCE_MS before the
 * filtered state reports blocked, but for the much longer CLEAR_DEBOUNCE_MS before it reports
 * clear. That asymmetry is the whole point - the balls have pickleball holes, so a seated ball
 * will briefly read clear when a hole lines up with the beam, and a long clear debounce rides
 * through that glitch while a short blocked debounce keeps arrivals responsive.
 *
 * Use {@link #isBlocked()} for logic and {@link #isRawBlocked()} only for diagnostics.
 */
public class BeamBreakSensor {

    private final BeamBreakPosition position;
    private final AnalogInput analogInput;

    private double lastVoltage;
    private double minVoltageSeen = Double.POSITIVE_INFINITY;
    private double maxVoltageSeen = Double.NEGATIVE_INFINITY;

    private boolean rawBlocked;
    private boolean filteredBlocked;
    private boolean initialized;

    /** Raw state currently waiting out its debounce; equals filteredBlocked when nothing is pending. */
    private boolean pendingState;
    private long pendingSinceMs;
    private long lastFilteredChangeMs;

    private int filteredTransitionCount;
    private int rejectedGlitchCount;

    public BeamBreakSensor(OpModeUtilities opModeUtilities, BeamBreakPosition position) {
        this(position, opModeUtilities.getHardwareMap().get(AnalogInput.class, position.getDeviceName()));
    }

    /** Direct-injection constructor, for a sensor you have already pulled out of the hardware map. */
    public BeamBreakSensor(BeamBreakPosition position, AnalogInput analogInput) {
        this.position = position;
        this.analogInput = analogInput;
    }

    /** Reads the sensor and advances both filter stages. Call once per loop. */
    public void update() {
        update(System.currentTimeMillis());
    }

    public void update(long nowMs) {
        update(analogInput.getVoltage(), nowMs);
    }

    /**
     * Advances the filters from a voltage you supply instead of reading the hardware. Exposed so
     * the filter can be driven from recorded or synthetic data with an explicit clock.
     */
    public void update(double voltage, long nowMs) {
        lastVoltage = voltage;
        if (voltage < minVoltageSeen) {
            minVoltageSeen = voltage;
        }
        if (voltage > maxVoltageSeen) {
            maxVoltageSeen = voltage;
        }

        updateRawState(voltage);

        if (!initialized) {
            // Seed the filter from the first reading instead of debouncing up from a guess, so the
            // count is correct on the very first loop of the OpMode.
            filteredBlocked = rawBlocked;
            pendingState = rawBlocked;
            pendingSinceMs = nowMs;
            lastFilteredChangeMs = nowMs;
            initialized = true;
            return;
        }

        if (rawBlocked != pendingState) {
            if (pendingState != filteredBlocked) {
                // A change that was mid-debounce just reversed itself: that is the glitch this
                // filter exists to swallow.
                rejectedGlitchCount++;
            }
            pendingState = rawBlocked;
            pendingSinceMs = nowMs;
        }

        if (pendingState != filteredBlocked) {
            double requiredMs = pendingState
                    ? BeamBreakConfig.BLOCKED_DEBOUNCE_MS
                    : BeamBreakConfig.CLEAR_DEBOUNCE_MS;
            if (nowMs - pendingSinceMs >= requiredMs) {
                filteredBlocked = pendingState;
                lastFilteredChangeMs = nowMs;
                filteredTransitionCount++;
            }
        }
    }

    /** Schmitt trigger around this sensor's threshold, in whichever polarity is configured. */
    private void updateRawState(double voltage) {
        // Positive means "looks blocked", regardless of whether blocked is the low or high rail.
        double blockedness = BeamBreakConfig.BLOCKED_IS_LOW_VOLTAGE
                ? getThresholdVolts() - voltage
                : voltage - getThresholdVolts();
        double halfBand = BeamBreakConfig.HYSTERESIS_V / 2.0;

        if (!initialized) {
            rawBlocked = blockedness > 0;
        } else if (rawBlocked) {
            if (blockedness < -halfBand) {
                rawBlocked = false;
            }
        } else {
            if (blockedness > halfBand) {
                rawBlocked = true;
            }
        }
    }

    public double getThresholdVolts() {
        switch (position) {
            case A_OUTER:
                return BeamBreakConfig.THRESHOLD_V_A_OUTER;
            case A_INNER:
                return BeamBreakConfig.THRESHOLD_V_A_INNER;
            case B_INNER:
                return BeamBreakConfig.THRESHOLD_V_B_INNER;
            case B_OUTER:
                return BeamBreakConfig.THRESHOLD_V_B_OUTER;
            default:
                throw new IllegalStateException("No threshold configured for " + position);
        }
    }

    /** Debounced state - the one to use for counting and for any robot logic. */
    public boolean isBlocked() {
        return filteredBlocked;
    }

    /** Pre-debounce state, after hysteresis. Diagnostics only. */
    public boolean isRawBlocked() {
        return rawBlocked;
    }

    /** True while a raw change is still waiting out its debounce window. */
    public boolean isChangePending() {
        return initialized && pendingState != filteredBlocked;
    }

    /**
     * True when the reading is outside the window a healthy sensor should stay in, which usually
     * means the sensor is unplugged or its wiring is broken. A faulted sensor still reports a
     * filtered state; BeamBreakArray just stops letting it veto the plausibility pass.
     */
    public boolean isFaulted() {
        return initialized
                && (lastVoltage < BeamBreakConfig.MIN_VALID_V || lastVoltage > BeamBreakConfig.MAX_VALID_V);
    }

    public long getMsInFilteredState(long nowMs) {
        return initialized ? nowMs - lastFilteredChangeMs : 0;
    }

    public long getMsInFilteredState() {
        return getMsInFilteredState(System.currentTimeMillis());
    }

    public BeamBreakPosition getPosition() {
        return position;
    }

    public double getLastVoltage() {
        return lastVoltage;
    }

    /** Lowest voltage seen since init - use it with the max to pick this sensor's threshold. */
    public double getMinVoltageSeen() {
        return minVoltageSeen == Double.POSITIVE_INFINITY ? 0 : minVoltageSeen;
    }

    public double getMaxVoltageSeen() {
        return maxVoltageSeen == Double.NEGATIVE_INFINITY ? 0 : maxVoltageSeen;
    }

    /** Suggested threshold for this sensor: the midpoint of the voltage range actually observed. */
    public double getSuggestedThresholdVolts() {
        return (getMinVoltageSeen() + getMaxVoltageSeen()) / 2.0;
    }

    /** How many times the debounced state has actually changed. */
    public int getFilteredTransitionCount() {
        return filteredTransitionCount;
    }

    /**
     * How many raw flips reversed before finishing their debounce. A climbing count here while
     * balls sit still is the hole-glitch rate, and is the signal to raise CLEAR_DEBOUNCE_MS.
     */
    public int getRejectedGlitchCount() {
        return rejectedGlitchCount;
    }

    public boolean isInitialized() {
        return initialized;
    }

    /** Drops all filter state so the next update() reseeds from the live reading. */
    public void reset() {
        initialized = false;
        filteredTransitionCount = 0;
        rejectedGlitchCount = 0;
        minVoltageSeen = Double.POSITIVE_INFINITY;
        maxVoltageSeen = Double.NEGATIVE_INFINITY;
    }

    @Override
    public String toString() {
        return String.format("%s: %.2fV raw=%s filtered=%s%s",
                position.shortLabel(), lastVoltage, rawBlocked, filteredBlocked,
                isFaulted() ? " FAULT" : "");
    }
}
