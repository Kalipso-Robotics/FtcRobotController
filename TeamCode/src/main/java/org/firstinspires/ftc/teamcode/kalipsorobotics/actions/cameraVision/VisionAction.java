package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;

import java.util.function.Predicate;
import java.util.function.Supplier;

/**
 * Generic action that waits for any vision result to satisfy a condition.
 *
 * Works with any KVisionProcessor, KRoboflowDetector, or any other source
 * that can supply a result — just pass a method reference as the supplier.
 *
 * USAGE — wait for a purple artifact to appear:
 *
 *   ArtifactDetectionProcessor artifacts = ...;
 *   VisionAction<DetectedBlob> waitForArtifact = new VisionAction<>(
 *       artifacts::getLargestPurpleBlob,   // supplier
 *       blob -> blob != null,              // done when a blob is found
 *       3.0                                // timeout in seconds
 *   );
 *   actionRunner.run(waitForArtifact);
 *   DetectedBlob target = waitForArtifact.getFinalResult();
 *
 * USAGE — wait for a Roboflow detector to find something:
 *
 *   YellowRectDetector detector = ...;
 *   VisionAction<List<YellowRect>> waitForRect = new VisionAction<>(
 *       detector::getLatestResult,
 *       rects -> !rects.isEmpty(),
 *       4.0
 *   );
 *
 * THREADING:
 *   update() runs on the robot main thread. The supplier reads from a volatile
 *   field in the processor (written by the camera thread) — non-blocking.
 *
 * @param <T> The result type produced by the vision processor.
 */
public class VisionAction<T> extends Action {

    private final Supplier<T> resultSupplier;
    private final Predicate<T> completionCondition;
    private final long timeoutMs;

    private long startTimeMs = -1;
    private T finalResult;
    private boolean timedOut = false;

    /**
     * @param resultSupplier      Method reference that returns the latest detection
     *                            (e.g. artifacts::getLargestPurpleBlob).
     * @param completionCondition Returns true when the result is good enough to proceed.
     * @param timeoutSeconds      How long to wait before giving up and marking done.
     */
    public VisionAction(Supplier<T> resultSupplier,
                        Predicate<T> completionCondition,
                        double timeoutSeconds) {
        this.resultSupplier = resultSupplier;
        this.completionCondition = completionCondition;
        this.timeoutMs = (long)(timeoutSeconds * 1000);
    }

    @Override
    protected void update() {
        if (startTimeMs < 0) {
            startTimeMs = System.currentTimeMillis();
        }

        T result = resultSupplier.get();
        if (result != null && completionCondition.test(result)) {
            finalResult = result;
            isDone = true;
            return;
        }

        if (System.currentTimeMillis() - startTimeMs >= timeoutMs) {
            timedOut = true;
            isDone = true;
        }
    }

    /** The result that satisfied the completion condition. Null if timed out. */
    public T getFinalResult() { return finalResult; }

    /** True if the action finished because time ran out rather than a detection. */
    public boolean hasTimedOut() { return timedOut; }
}
