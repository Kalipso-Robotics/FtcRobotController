package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak;

/**
 * An immutable snapshot of what the beam breaks saw on one update, published to SharedData so
 * anything that cares about the ball count can subscribe without holding the sensors.
 *
 * Every field is captured in the same pass, and the whole object is swapped into SharedData
 * behind a single volatile write. That is the point of making this a value type: a subscriber can
 * never catch a count from one loop next to positions from the next one, even reading from
 * another thread. Occupancy is packed into bitmasks so a snapshot costs one small allocation per
 * loop and the accessors allocate nothing.
 *
 * Instances come from {@link BeamBreakArray#snapshot()}; the rest of the robot only reads them.
 */
public final class BeamBreakData {

    /**
     * What subscribers see before anything has been published. Reports an empty, unreliable robot
     * with no timestamp, so code that checks {@link #hasData()} or {@link #isReliable()} cannot
     * mistake "we have not looked yet" for "there are no balls".
     */
    public static final BeamBreakData UNKNOWN =
            new BeamBreakData(0, 0, 0, 0, 0, true, false, false, 0);

    private final int count;
    private final int measuredCount;
    private final int blockedMask;
    private final int measuredMask;
    private final int correctionCount;
    private final boolean patternPlausible;
    private final boolean settling;
    private final boolean reliable;
    private final long timestampMs;

    BeamBreakData(int count, int measuredCount, int blockedMask, int measuredMask,
                  int correctionCount, boolean patternPlausible, boolean settling,
                  boolean reliable, long timestampMs) {
        this.count = count;
        this.measuredCount = measuredCount;
        this.blockedMask = blockedMask;
        this.measuredMask = measuredMask;
        this.correctionCount = correctionCount;
        this.patternPlausible = patternPlausible;
        this.settling = settling;
        this.reliable = reliable;
        this.timestampMs = timestampMs;
    }

    /** Balls in the robot, filtered and plausibility-corrected. This is the number to use. */
    public int getCount() {
        return count;
    }

    /** Debounced count with no plausibility pass applied, for diagnostics. */
    public int getMeasuredCount() {
        return measuredCount;
    }

    public int getCapacity() {
        return BeamBreakPosition.COUNT;
    }

    /** Occupancy at one position, after the plausibility pass. */
    public boolean isBlocked(BeamBreakPosition position) {
        return (blockedMask & (1 << position.index())) != 0;
    }

    /** Occupancy at one position straight from its debounced sensor, uncorrected. */
    public boolean isMeasuredBlocked(BeamBreakPosition position) {
        return (measuredMask & (1 << position.index())) != 0;
    }

    /** True when this position read clear but was counted as a ball by the plausibility pass. */
    public boolean isInferredBlocked(BeamBreakPosition position) {
        return isBlocked(position) && !isMeasuredBlocked(position);
    }

    public boolean isFull() {
        return count >= BeamBreakPosition.COUNT;
    }

    public boolean isEmpty() {
        return count == 0;
    }

    /** True when the debounced readings were already a physically possible pattern. */
    public boolean isPatternPlausible() {
        return patternPlausible;
    }

    /** True while a sensor changed too recently for the plausibility pass to run. */
    public boolean isSettling() {
        return settling;
    }

    /** How many positions the plausibility pass flipped from clear to blocked. */
    public int getCorrectionCount() {
        return correctionCount;
    }

    /**
     * False when a sensor looked electrically broken, or when nothing has been published yet.
     * The plausibility pass cannot rescue a sensor stuck reading blocked, so gate match logic on
     * this rather than trusting the count blindly.
     */
    public boolean isReliable() {
        return reliable;
    }

    /** System.currentTimeMillis() when this snapshot was taken; 0 for {@link #UNKNOWN}. */
    public long getTimestampMs() {
        return timestampMs;
    }

    /** False only for {@link #UNKNOWN} - nothing has polled the sensors yet this OpMode. */
    public boolean hasData() {
        return timestampMs > 0;
    }

    public long getAgeMs() {
        return hasData() ? System.currentTimeMillis() - timestampMs : Long.MAX_VALUE;
    }

    /**
     * True when this snapshot is recent enough to act on. Statics outlive an OpMode in FTC, so a
     * subscriber that runs before the first publish of a new match would otherwise read last
     * match's count - an age check rejects that without needing an explicit reset.
     */
    public boolean isFresh(long maxAgeMs) {
        return hasData() && getAgeMs() <= maxAgeMs;
    }

    /**
     * Occupancy in physical A-to-B order: {@code X} measured blocked, {@code +} inferred blocked
     * by the plausibility pass, {@code .} clear.
     */
    public String getOccupancyString() {
        StringBuilder builder = new StringBuilder(BeamBreakPosition.COUNT + 2);
        builder.append('[');
        for (BeamBreakPosition position : BeamBreakPosition.values()) {
            if (isMeasuredBlocked(position)) {
                builder.append('X');
            } else if (isBlocked(position)) {
                builder.append('+');
            } else {
                builder.append('.');
            }
        }
        builder.append(']');
        return builder.toString();
    }

    @Override
    public String toString() {
        if (!hasData()) {
            return "BeamBreakData{no data}";
        }
        return "BeamBreakData{" + count + "/" + getCapacity() + " " + getOccupancyString()
                + (reliable ? "" : " UNRELIABLE") + (settling ? " settling" : "") + "}";
    }
}
