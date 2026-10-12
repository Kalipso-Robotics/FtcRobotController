package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.beambreak;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakArray;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * Polls the beam break sensors and publishes the ball count to SharedData.
 *
 * This is the publisher. Exactly one of these owns the sensors for the whole OpMode, and every
 * update pushes a fresh {@link BeamBreakData} snapshot into
 * {@link SharedData#getBeamBreakData()}. Anything else that wants the count subscribes there
 * instead of holding a reference to this action:
 *
 * <pre>
 *   // publisher, in the OpMode - init
 *   beamBreakCount = new BeamBreakCountAction(opModeUtilities);
 *   beamBreakCount.setName("beamBreakCount");
 *
 *   // publisher - once per loop
 *   beamBreakCount.updateCheckDone();
 *
 *   // any subscriber, anywhere, including off the main thread
 *   BeamBreakData balls = SharedData.getBeamBreakData();
 *   if (balls.isFresh(250) && balls.isReliable() &amp;&amp; balls.isFull()) { ... }
 * </pre>
 *
 * The action never finishes - it is a continuous sensor poller, like ShooterRun. If it stops being
 * updated the snapshot in SharedData simply goes stale, which subscribers detect with
 * {@link BeamBreakData#isFresh(long)} rather than silently reading a frozen count.
 */
public class BeamBreakCountAction extends Action {

    private final BeamBreakArray array;

    public BeamBreakCountAction(OpModeUtilities opModeUtilities) {
        this(new BeamBreakArray(opModeUtilities));
    }

    /** Shares an existing array, for when something else already owns the sensors. */
    public BeamBreakCountAction(BeamBreakArray array) {
        this.array = array;
        this.name = "BeamBreakCountAction";
    }

    @Override
    protected void update() {
        if (!hasStarted) {
            hasStarted = true;
        }
        array.update();
    }

    /**
     * Never done. A sensor poller that completed would silently freeze the count at a stale
     * value, which is worse than useless, so it stays running for the whole OpMode.
     */
    @Override
    protected boolean isUpdateDone() {
        return false;
    }

    /** Balls in the robot, filtered and plausibility-corrected. */
    public int getCount() {
        return array.getCount();
    }

    /** Uncorrected debounced count, for comparing against {@link #getCount()} on the field. */
    public int getMeasuredCount() {
        return array.getMeasuredCount();
    }

    public boolean isFull() {
        return array.isFull();
    }

    public boolean isEmpty() {
        return array.isEmpty();
    }

    public boolean isBlocked(BeamBreakPosition position) {
        return array.isBlocked(position);
    }

    /** False when a sensor looks electrically broken - see BeamBreakArray.isCountReliable(). */
    public boolean isCountReliable() {
        return array.isCountReliable();
    }

    public BeamBreakArray getArray() {
        return array;
    }

    /** The snapshot this action just published. Same object SharedData is serving. */
    public BeamBreakData getSnapshot() {
        return array.getSnapshot();
    }

    /** One-line summary for driver station telemetry. */
    public String getTelemetryLine() {
        String line = array.getCount() + "/" + array.getCapacity() + " " + array.getOccupancyString();
        if (!array.isCountReliable()) {
            line += " SENSOR FAULT " + array.getFaultedPositions();
        } else if (!array.isPatternPlausible()) {
            line += " (corrected)";
        }
        return line;
    }

    /** Clears filter state, e.g. after a jam clear where you want the count reseeded. */
    public void resetFilters() {
        KLog.d("BeamBreak", "Filters reset");
        array.reset();
    }
}
