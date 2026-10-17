package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;

import java.util.function.Supplier;

/**
 * Generic action that waits until a vision source gives a result (anything that is not null).
 *
 * Works with any KVisionProcessor, KRoboflowDetector, or any other source
 * that can supply a result — just pass a method reference as the supplier.
 * The supplier returns null while there is nothing good yet. If you need a condition
 * (for example "inside the zone"), put it in the supplier and return null when it is not met.
 *
 * USAGE — wait for a purple artifact to appear:
 *
 *   ArtifactDetectionProcessor artifacts = ...;
 *   VisionAction<DetectedBlob> waitForArtifact = new VisionAction<>(
 *       artifacts::getLargestPurpleBlob,   // supplier: null until a blob is found
 *       3.0                                // timeout in seconds
 *   );
 *   actionRunner.run(waitForArtifact);
 *   DetectedBlob target = waitForArtifact.getFinalResult();
 *
 * USAGE — wait for a Roboflow detector to find something:
 *
 *   YellowRectDetector detector = ...;
 *   VisionAction<List<YellowRect>> waitForRect = new VisionAction<>(
 *       () -> { List<YellowRect> r = detector.getLatestResult(); return r.isEmpty() ? null : r; },
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
    private final long timeoutMs;

    private long startTimeMs = -1;
    private T finalResult;
    private boolean timedOut = false;

    /**
     * @param resultSupplier Method reference that returns the latest detection, or null if
     *                       there is none yet (e.g. artifacts::getLargestPurpleBlob).
     * @param timeoutSeconds How long to wait before giving up and marking done.
     */
    public VisionAction(Supplier<T> resultSupplier, double timeoutSeconds) {
        this.resultSupplier = resultSupplier;
        this.timeoutMs = (long)(timeoutSeconds * 1000);
    }

    @Override
    protected void update() {
        if (startTimeMs < 0) {
            startTimeMs = System.currentTimeMillis();
        }

        T result = resultSupplier.get();
        if (result != null) {
            finalResult = result;
            isDone = true;
            return;
        }

        if (System.currentTimeMillis() - startTimeMs >= timeoutMs) {
            timedOut = true;
            isDone = true;
        }
    }

    /** The first result the supplier gave that was not null. Null if timed out. */
    public T getFinalResult() { return finalResult; }

    /** True if the action finished because time ran out rather than a detection. */
    public boolean hasTimedOut() { return timedOut; }
}
