package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.beambreak;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakArray;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * Finishes once the ball count satisfies a comparison, so it can gate other actions in a
 * KActionSet - "intake until we are holding three", or "wait until the robot is empty after
 * shooting".
 *
 * Comes in two flavours:
 *
 * <ul>
 *   <li>{@link #subscribing} reads the count from SharedData. Nothing to thread through, but it
 *       needs a BeamBreakCountAction running somewhere to publish, and it refuses to act on a
 *       snapshot older than {@link #MAX_DATA_AGE_MS} - so if the publisher is missing this waits
 *       and then times out rather than trusting a stale count.</li>
 *   <li>The array constructors poll the sensors themselves, which is self-sufficient for an auto
 *       path that does not otherwise own a BeamBreakCountAction. Safe to run alongside one on the
 *       same array: the filters are driven by absolute timestamps, so two updates in one loop with
 *       the same readings are idempotent and the debounce does not skew.</li>
 * </ul>
 *
 * <pre>
 *   WaitForBallCountAction waitForThree =
 *           WaitForBallCountAction.subscribing(Comparison.AT_LEAST, 3, 4000);
 *   waitForThree.setName("waitForThreeBalls");
 *   stopIntake.setDependentActions(waitForThree);
 * </pre>
 */
public class WaitForBallCountAction extends Action {

    public enum Comparison {
        AT_LEAST,
        AT_MOST,
        EXACTLY
    }

    /** Default settling time: the count has to hold for this long before the action finishes. */
    public static final double DEFAULT_REQUIRED_STABLE_MS = 50;

    /**
     * Oldest SharedData snapshot a subscribing instance will believe. A publisher runs once per
     * loop, so anything older than this means nobody is polling the sensors.
     */
    public static final long MAX_DATA_AGE_MS = 250;

    /** Null when this instance subscribes to SharedData instead of owning the sensors. */
    private final BeamBreakArray array;
    private final Comparison comparison;
    private final int targetCount;
    private final double timeoutMs;
    private final double requiredStableMs;

    private final ElapsedTime actionTimer = new ElapsedTime();
    private final ElapsedTime satisfiedTimer = new ElapsedTime();

    private boolean wasSatisfied = false;
    private boolean timedOut = false;
    private boolean sawStaleData = false;

    /** Subscribes to SharedData, with no timeout. Needs a publisher; see the class docs. */
    public static WaitForBallCountAction subscribing(Comparison comparison, int targetCount) {
        return new WaitForBallCountAction(null, comparison, targetCount, 0,
                DEFAULT_REQUIRED_STABLE_MS);
    }

    /** Subscribes to SharedData, giving up after timeoutMs. Pass 0 or less for no timeout. */
    public static WaitForBallCountAction subscribing(Comparison comparison, int targetCount,
                                                    double timeoutMs) {
        return new WaitForBallCountAction(null, comparison, targetCount, timeoutMs,
                DEFAULT_REQUIRED_STABLE_MS);
    }

    public static WaitForBallCountAction subscribing(Comparison comparison, int targetCount,
                                                    double timeoutMs, double requiredStableMs) {
        return new WaitForBallCountAction(null, comparison, targetCount, timeoutMs,
                requiredStableMs);
    }

    /** Polls the array itself, with no timeout: waits indefinitely for the count to be reached. */
    public WaitForBallCountAction(BeamBreakArray array, Comparison comparison, int targetCount) {
        this(array, comparison, targetCount, 0, DEFAULT_REQUIRED_STABLE_MS);
    }

    /** Polls the array itself, giving up after timeoutMs; pass 0 or less for no timeout. */
    public WaitForBallCountAction(BeamBreakArray array, Comparison comparison, int targetCount,
                                  double timeoutMs) {
        this(array, comparison, targetCount, timeoutMs, DEFAULT_REQUIRED_STABLE_MS);
    }

    public WaitForBallCountAction(BeamBreakArray array, Comparison comparison, int targetCount,
                                  double timeoutMs, double requiredStableMs) {
        this.array = array;
        this.comparison = comparison;
        this.targetCount = targetCount;
        this.timeoutMs = timeoutMs;
        this.requiredStableMs = requiredStableMs;
        this.name = "WaitForBallCount_" + comparison + "_" + targetCount;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }

        if (!hasStarted) {
            actionTimer.reset();
            satisfiedTimer.reset();
            hasStarted = true;
        }

        if (array != null) {
            array.update();
        }

        BeamBreakData data = getData();
        boolean usable = array != null || data.isFresh(MAX_DATA_AGE_MS);
        if (!usable && !sawStaleData) {
            sawStaleData = true;
            KLog.e("BeamBreak_Wait", "[" + getName() + "] no fresh beam break data in SharedData - "
                    + "is a BeamBreakCountAction being updated? Waiting instead of guessing.");
        }

        int count = data.getCount();
        boolean satisfied = usable && isSatisfied(count);

        if (satisfied && !wasSatisfied) {
            satisfiedTimer.reset();
        }
        wasSatisfied = satisfied;

        if (satisfied && satisfiedTimer.milliseconds() >= requiredStableMs) {
            isDone = true;
            KLog.d("BeamBreak_Wait", () -> String.format(
                    "[%s] satisfied at count=%d after %.0f ms", getName(), count,
                    actionTimer.milliseconds()));
            return;
        }

        if (timeoutMs > 0 && actionTimer.milliseconds() > timeoutMs) {
            timedOut = true;
            isDone = true;
            KLog.d("BeamBreak_Wait", () -> String.format(
                    "[%s] TIMED OUT after %.0f ms at count=%d (wanted %s %d)%s", getName(),
                    actionTimer.milliseconds(), count, comparison, targetCount,
                    usable ? "" : " - data was stale"));
            return;
        }

        KLog.d("BeamBreak_Wait", () -> String.format("[%s] waiting, count=%d %s",
                getName(), count, data.getOccupancyString()));
    }

    private BeamBreakData getData() {
        return array != null ? array.getSnapshot() : SharedData.getBeamBreakData();
    }

    private boolean isSatisfied(int count) {
        switch (comparison) {
            case AT_LEAST:
                return count >= targetCount;
            case AT_MOST:
                return count <= targetCount;
            case EXACTLY:
                return count == targetCount;
            default:
                return false;
        }
    }

    /**
     * True when this action finished because it ran out of time rather than because the count was
     * reached. Check it before treating completion as success.
     */
    public boolean isTimedOut() {
        return timedOut;
    }

    /** True if this action ever went a cycle without fresh data. Points at a missing publisher. */
    public boolean sawStaleData() {
        return sawStaleData;
    }

    public int getCount() {
        return getData().getCount();
    }

    /** Null when this instance subscribes to SharedData rather than owning the sensors. */
    public BeamBreakArray getArray() {
        return array;
    }
}
