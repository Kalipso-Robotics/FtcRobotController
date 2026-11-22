package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.BeamBreakConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import java.util.ArrayList;
import java.util.List;

/**
 * Every beam break sensor in BeamBreakConfig.SENSORS together (any count, ordered A end to B end), plus the physical-plausibility pass that makes the count
 * robust to a single ball's hole lining up with a beam.
 *
 * Every {@link #update(long)} ends by publishing an immutable {@link BeamBreakData} snapshot to
 * {@link SharedData}, so subscribers never need this object. Publishing lives here rather than in
 * the action so that any path which updates the array keeps SharedData current - the same way
 * Odometry publishes its pose. This class's own getters read back through that snapshot, so the
 * array and its subscribers cannot disagree.
 *
 * <h3>Why a pattern check works here</h3>
 * The intake is double sided and balls stack from the outside in, so the occupied positions are
 * always a block growing inward from the A end plus a block growing inward from the B end. That
 * means the EMPTY positions are always ONE contiguous run in the middle - never two. So a
 * reading like {@code X . X .} (blocked, clear, blocked, clear) has two separate empty runs and
 * cannot happen on real hardware.
 *
 * The only false reading these sensors produce is a ball reading CLEAR through its own hole; a
 * beam does not report blocked with nothing in front of it. So whenever the pattern is
 * impossible, the fix is always to flip some clear readings to blocked, never the reverse. This
 * class keeps the LONGEST run of clear readings as genuinely empty and counts every remaining
 * clear position as a ball, which is the smallest correction that restores a possible pattern:
 *
 * <pre>
 *   count = min(capacity, sensors - (length of the longest contiguous run of clear readings))
 * </pre>
 *
 * Each blocked sensor counts as one ball, clamped to capacity, so this assumes sensors are spaced
 * at least a ball apart.
 *
 * The count that formula produces does not depend on which of two equally long runs you keep, so
 * the total is unambiguous even when the per-position assignment is a coin flip. Ties are broken
 * toward the run that has been stably clear the longest, since a position that just flipped to
 * clear is the likelier glitch.
 *
 * <h3>Limits worth knowing</h3>
 * This recovers any single false clear, and any number of false clears that are not adjacent to a
 * genuine empty run. It cannot recover a case where every position reads clear at once, which is
 * why the sensors should be spaced so no two can be holed simultaneously. It also cannot help
 * with a sensor that reads permanently blocked (an unplugged one often does), so check
 * {@link #isCountReliable()} before trusting the count in match logic.
 *
 * The pattern argument only holds for balls at rest. A ball still moving inward blocks an inner
 * beam while the outer beam it just passed reads clear, which is one of those "impossible"
 * patterns, and correcting it would count a ball that is only in transit. So the pass is gated on
 * every sensor having been stable for CORRECTION_SETTLE_MS; while it is not, the count is the
 * plain debounced one and {@link #isSettling()} is true.
 */
public class BeamBreakArray {

    public static final String TAG = "Robot_BeamBreak";

    /** In physical order, index 0 at the A end. */
    private final List<BeamBreakSensor> sensors;
    private final int capacity;

    /** Debounced per-sensor readings, before the pattern pass. Indexed by physical position. */
    private final boolean[] measuredBlocked;
    /** Readings after the pattern pass - what getCount() and isBlocked() report. */
    private final boolean[] correctedBlocked;

    private int count;
    private int measuredCount;
    private boolean patternPlausible = true;
    private int correctionCount;
    private boolean settling;
    private boolean wasReliable = true;

    /**
     * The last published snapshot. Every public reader below goes through this, so the array and
     * its SharedData subscribers can never disagree about what the sensors said.
     */
    private volatile BeamBreakData lastSnapshot = BeamBreakData.UNKNOWN;

    /** Builds every sensor in BeamBreakConfig.SENSORS from the hardware map. */
    public BeamBreakArray(OpModeUtilities opModeUtilities) {
        this(sensorsFromConfig(opModeUtilities), BeamBreakConfig.CAPACITY);
    }

    /** Direct-injection constructor for sensors you built yourself, in A-to-B order. */
    public BeamBreakArray(List<BeamBreakSensor> sensorList, int capacity) {
        // Occupancy is packed into an int bitmask in BeamBreakData.
        if (sensorList.isEmpty() || sensorList.size() >= Integer.SIZE) {
            throw new IllegalArgumentException("BeamBreakArray needs 1-31 sensors, got " + sensorList.size());
        }
        this.sensors = new ArrayList<>(sensorList);
        this.capacity = capacity;
        this.measuredBlocked = new boolean[sensors.size()];
        this.correctedBlocked = new boolean[sensors.size()];
    }

    private static List<BeamBreakSensor> sensorsFromConfig(OpModeUtilities opModeUtilities) {
        List<BeamBreakSensor> list = new ArrayList<>();
        for (BeamBreakPosition position : BeamBreakConfig.SENSORS) {
            list.add(new BeamBreakSensor(opModeUtilities, position));
        }
        return list;
    }

    /** Reads every sensor, runs both filter stages, then runs the pattern pass. Call once per loop. */
    public void update() {
        update(System.currentTimeMillis());
    }

    public void update(long nowMs) {
        int previousCount = count;
        measuredCount = 0;
        for (int index = 0; index < sensors.size(); index++) {
            BeamBreakSensor sensor = sensors.get(index);
            sensor.update(nowMs);
            boolean blocked = sensor.isBlocked();
            measuredBlocked[index] = blocked;
            if (blocked) {
                measuredCount++;
            }
        }

        applyPatternCorrection(nowMs);

        lastSnapshot = buildSnapshot(nowMs);
        SharedData.setBeamBreakData(lastSnapshot);

        boolean reliable = lastSnapshot.isReliable();
        if (wasReliable && !reliable) {
            KLog.e(TAG, "fault=unreliable faulted=" + getFaultedPositions() + " pattern=" + getOccupancyString());
        }
        wasReliable = reliable;
        if (count != previousCount) {
            KLog.d(TAG, () -> String.format("countChanged=%d->%d pattern=%s", previousCount, count, getOccupancyString()));
        }
        if (KLog.isDebug()) {
            KLog.d(TAG, () -> String.format(
                    "count=%d measured=%d plausible=%s settling=%s corrections=%d pattern=%s",
                    count, measuredCount, patternPlausible, settling, correctionCount, getOccupancyString()));
        }
    }

    /** Packs this update's results into the immutable snapshot that subscribers read. */
    private BeamBreakData buildSnapshot(long nowMs) {
        int blockedMask = 0;
        int measuredMask = 0;
        for (int index = 0; index < sensors.size(); index++) {
            if (correctedBlocked[index]) {
                blockedMask |= 1 << index;
            }
            if (measuredBlocked[index]) {
                measuredMask |= 1 << index;
            }
        }
        return new BeamBreakData(count, measuredCount, capacity, sensors.size(), blockedMask, measuredMask, correctionCount,
                patternPlausible, settling, isCountReliable(), nowMs);
    }

    /**
     * Keeps the longest contiguous run of clear readings as genuinely empty and treats every other
     * clear reading as a ball whose hole fooled its beam. When the reading is already physically
     * possible this is a no-op, because there is at most one clear run to keep.
     */
    private void applyPatternCorrection(long nowMs) {
        int n = sensors.size();
        System.arraycopy(measuredBlocked, 0, correctedBlocked, 0, n);
        patternPlausible = countClearRuns() <= 1;
        correctionCount = 0;
        settling = !isSettled(nowMs);

        if (!BeamBreakConfig.ENABLE_PATTERN_CORRECTION || settling) {
            // Mid-motion the readings are allowed to look impossible, so take them at face value
            // rather than inventing a ball that is only in transit. See CORRECTION_SETTLE_MS.
            count = Math.min(capacity, measuredCount);
            return;
        }

        int bestStart = -1;
        int bestLength = 0;
        long bestStability = Long.MIN_VALUE;

        int i = 0;
        while (i < n) {
            if (measuredBlocked[i]) {
                i++;
                continue;
            }
            int runStart = i;
            // How long has the least-settled position in this run been reading clear? A run
            // containing a position that just flipped is the likelier glitch.
            long runStability = Long.MAX_VALUE;
            while (i < n && !measuredBlocked[i]) {
                long msClear = sensors.get(i).getMsInFilteredState(nowMs);
                runStability = Math.min(runStability, msClear);
                i++;
            }
            int runLength = i - runStart;
            if (runLength > bestLength || (runLength == bestLength && runStability > bestStability)) {
                bestStart = runStart;
                bestLength = runLength;
                bestStability = runStability;
            }
        }

        for (int index = 0; index < n; index++) {
            boolean insideKeptRun = bestStart >= 0 && index >= bestStart && index < bestStart + bestLength;
            if (!measuredBlocked[index] && !insideKeptRun) {
                correctedBlocked[index] = true;
                correctionCount++;
            }
        }

        count = Math.min(capacity, n - bestLength);

        if (correctionCount > 0 && KLog.isDebug()) {
            final int corrected = correctionCount;
            KLog.d(TAG, () -> String.format("impossiblePattern=%s inferredBalls=%d count=%d",
                    renderPattern(measuredBlocked), corrected, count));
        }
    }

    /** True once every sensor has held its filtered state for at least CORRECTION_SETTLE_MS. */
    private boolean isSettled(long nowMs) {
        for (BeamBreakSensor sensor : sensors) {
            if (sensor.getMsInFilteredState(nowMs) < BeamBreakConfig.CORRECTION_SETTLE_MS) {
                return false;
            }
        }
        return true;
    }

    private int countClearRuns() {
        int runs = 0;
        boolean inRun = false;
        for (int index = 0; index < sensors.size(); index++) {
            if (!measuredBlocked[index]) {
                if (!inRun) {
                    runs++;
                    inRun = true;
                }
            } else {
                inRun = false;
            }
        }
        return runs;
    }

    /**
     * The latest snapshot, identical to what SharedData is serving. Prefer
     * SharedData.getBeamBreakData() anywhere that does not already own this array.
     */
    public BeamBreakData getSnapshot() {
        return lastSnapshot;
    }

    /**
     * How many balls are in the robot, after filtering and the plausibility pass. This is the
     * number to use.
     */
    public int getCount() {
        return lastSnapshot.getCount();
    }

    /** Debounced count with no plausibility pass - for comparing against {@link #getCount()}. */
    public int getMeasuredCount() {
        return lastSnapshot.getMeasuredCount();
    }

    /** True when the debounced readings were already a physically possible pattern. */
    public boolean isPatternPlausible() {
        return lastSnapshot.isPatternPlausible();
    }

    /** How many positions the plausibility pass flipped from clear to blocked this update. */
    public int getCorrectionCount() {
        return lastSnapshot.getCorrectionCount();
    }

    /**
     * True while a sensor has changed too recently for the plausibility pass to run. The count is
     * the raw debounced one during this window, which is the right answer for balls in transit.
     */
    public boolean isSettling() {
        return lastSnapshot.isSettling();
    }

    /** Occupancy at one position, after the plausibility pass. */
    public boolean isBlocked(int index) {
        return lastSnapshot.isBlocked(index);
    }

    /** Occupancy at one position straight from its debounced sensor, uncorrected. */
    public boolean isMeasuredBlocked(int index) {
        return lastSnapshot.isMeasuredBlocked(index);
    }

    /** True when this position reads clear but was counted as a ball by the plausibility pass. */
    public boolean isInferredBlocked(int index) {
        return lastSnapshot.isInferredBlocked(index);
    }

    public boolean isFull() {
        return lastSnapshot.isFull();
    }

    public boolean isEmpty() {
        return lastSnapshot.isEmpty();
    }

    public int getCapacity() {
        return capacity;
    }

    /**
     * False when any sensor looks electrically broken. The plausibility pass cannot rescue a
     * sensor stuck reading blocked, so gate match logic on this rather than assuming the count.
     */
    public boolean isCountReliable() {
        for (BeamBreakSensor sensor : sensors) {
            if (sensor.isFaulted() || !sensor.isInitialized()) {
                return false;
            }
        }
        return true;
    }

    public List<BeamBreakPosition> getFaultedPositions() {
        List<BeamBreakPosition> faulted = new ArrayList<>();
        for (BeamBreakSensor sensor : sensors) {
            if (sensor.isFaulted()) {
                faulted.add(sensor.getPosition());
            }
        }
        return faulted;
    }

    public BeamBreakSensor getSensor(int index) {
        return sensors.get(index);
    }

    /** In physical order, index 0 at the A end. */
    public List<BeamBreakSensor> getSensors() {
        return sensors;
    }

    /**
     * Occupancy in physical A-to-B order for telemetry: {@code X} measured blocked, {@code +}
     * inferred blocked by the plausibility pass, {@code .} clear.
     */
    public String getOccupancyString() {
        return lastSnapshot.getOccupancyString();
    }

    private static String renderPattern(boolean[] blocked) {
        StringBuilder builder = new StringBuilder();
        for (boolean isBlocked : blocked) {
            builder.append(isBlocked ? 'X' : '.');
        }
        return builder.toString();
    }

    /** Drops all filter state so the next update() reseeds from live readings. */
    public void reset() {
        for (BeamBreakSensor sensor : sensors) {
            sensor.reset();
        }
        count = 0;
        measuredCount = 0;
        correctionCount = 0;
        patternPlausible = true;
        settling = false;
        wasReliable = true;
        lastSnapshot = BeamBreakData.UNKNOWN;
        SharedData.resetBeamBreakData();
    }

    @Override
    public String toString() {
        return "BeamBreakArray{count=" + count + " " + getOccupancyString() + "}";
    }
}
