package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.AprilTagConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AprilTagCamera;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.HiveTagLocalizer;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.LimelightAprilTagCamera;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.TagObservation;

import java.util.List;
import java.util.Locale;

/**
 * Turns HIVE tag sightings into (1) a robot field position for re-zeroing odometry and (2) the goal state (which of
 * our CELLs is up), both on SharedData. The math is HiveTagLocalizer; this class only feeds it and publishes.
 * Runs on its own thread (OpModeUtilities.runAprilTagExecutorService).
 *
 * The camera sits behind AprilTagCamera, so the Arducam (KAprilTagProcessor) can replace the Limelight:
 *   new AprilTagDetectionAction(opModeUtilities, turret, alliance, kAprilTagProcessor);
 *
 * One frame can be wrong (motion blur, a HIVE mid-swing), so both answers come from a sliding window of the last
 * AprilTagConfig.HIVE_WINDOW_FRAMES frames (none older than HIVE_WINDOW_MAX_AGE_MS): a CELL must win several frames,
 * the position is their average (kept as an offset from odometry, so it does not lag while driving).
 * setEnabled(false) turns the camera off between viewpoints.
 *
 * Heading comes from odometry (IMU), so a fix only moves x and y.
 * ponytail: uses the pose at processing time, not at frame capture; error ~ speed * Limelight latency (~40 ms).
 * Re-zero while slow, or look the pose up with SharedData.getOdometryWheelIMUPositionAt if that is ever not enough.
 */
public class AprilTagDetectionAction extends Action {
    public static final String TAG = "Vision_Limelight";

    private final OpModeUtilities opModeUtilities;
    private final AprilTagCamera camera;
    private final Turret turret;
    private final AllianceColor allianceColor;

    private boolean enabled = true; // the constructor starts the camera
    private long lastFrameId = -1;

    // Sliding window (ring buffer): slot nextSlot is overwritten next. Each slot is one camera frame.
    private final HiveTagLocalizer.Cell[] windowUpCell = new HiveTagLocalizer.Cell[AprilTagConfig.HIVE_WINDOW_FRAMES];
    private final boolean[] windowTipping = new boolean[AprilTagConfig.HIVE_WINDOW_FRAMES];
    private final boolean[] windowHasPosition = new boolean[AprilTagConfig.HIVE_WINDOW_FRAMES];
    // Position is kept as "how far odometry was off" (camera fix minus odometry at that frame), so it does not lag
    // behind while the robot drives: publishing adds the offset to where odometry is NOW.
    private final double[] windowOffsetX = new double[AprilTagConfig.HIVE_WINDOW_FRAMES];
    private final double[] windowOffsetY = new double[AprilTagConfig.HIVE_WINDOW_FRAMES];
    private final long[] windowNanos = new long[AprilTagConfig.HIVE_WINDOW_FRAMES]; // when the frame was handled
    private int nextSlot = 0;
    private int framesInWindow = 0;
    private int seenOdometryJumpCount = SharedData.getOdometryJumpCount();

    public AprilTagDetectionAction(OpModeUtilities opModeUtilities, Turret turret, AllianceColor allianceColor) {
        this(opModeUtilities, turret, allianceColor, new LimelightAprilTagCamera(opModeUtilities, allianceColor));
    }

    public AprilTagDetectionAction(OpModeUtilities opModeUtilities, Turret turret, AllianceColor allianceColor, AprilTagCamera camera) {
        this.opModeUtilities = opModeUtilities;
        this.turret = turret;
        this.allianceColor = allianceColor;
        this.camera = camera;
        this.camera.start();
    }

    public AprilTagCamera getCamera() {
        return camera;
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }

    /**
     * Turn the camera on or off (autos switch it off between viewpoints to save the Limelight and the CPU).
     * Only talks to the camera when the state really changes. Turning it back on forgets the old frames.
     * synchronized: called from the main loop while update() runs on the vision thread.
     */
    public synchronized void setEnabled(boolean enabled) {
        if (enabled == this.enabled) return;
        this.enabled = enabled;
        if (enabled) {
            clearWindow();
            camera.start();
        } else {
            camera.stop();
            SharedData.setHiveTipping(false); // nobody is watching any more: don't leave the HIVE blocked forever
        }
        KLog.d(TAG, () -> "event=camera enabled=" + enabled);
    }

    public synchronized boolean isEnabled() {
        return enabled;
    }

    @Override
    protected synchronized void update() {
        if (!enabled) return;

        // Odometry was reset: offsets measured before the jump would count the correction twice.
        int jumpCount = SharedData.getOdometryJumpCount();
        if (jumpCount != seenOdometryJumpCount) {
            seenOdometryJumpCount = jumpCount;
            clearWindow();
            SharedData.clearLimelightFix();
            KLog.d(TAG, "event=windowCleared reason=odometryJump");
        }

        // The thread polls every 10 ms but the Limelight is slower: skip a frame we already handled.
        long frameId = camera.getLatestFrameId();
        boolean cameraKnowsFrames = frameId != -1;
        if (cameraKnowsFrames && frameId == lastFrameId) return;
        lastFrameId = frameId;

        long now = System.nanoTime();
        List<TagObservation> observations = camera.getLatestObservations();
        if (!observations.isEmpty()) {
            Position odometry = SharedData.getOdometryWheelIMUPosition();
            HiveTagLocalizer.Fix fix = HiveTagLocalizer.solve(observations, odometry.getTheta(), turret.getAngleRad(), allianceColor);
            boolean frameSaysSomething = fix.tagsUsed > 0 || fix.tipping;
            if (frameSaysSomething) {
                // No turret encoder: mid-swing, the commanded angle is not where the camera points, so that position is not kept.
                boolean positionUsable = fix.robot != null && !turret.isMoving();
                addToWindow(fix, positionUsable, now, odometry);
                if (positionUsable) publishPosition(now, fix, odometry);
            }
        }
        // Every new frame, even one without our tags, so old frames (and an old "tipping") age out.
        publishGoalState(now);
    }

    // ---- Sliding window: the last HIVE_WINDOW_FRAMES frames, at most HIVE_WINDOW_MAX_AGE_MS old ----

    /** Put one frame into the window. Package-private for AprilTagDetectionActionWindowTest. */
    void addToWindow(HiveTagLocalizer.Fix fix, boolean positionUsable, long nowNanos, Position odometryAtFrame) {
        windowUpCell[nextSlot] = fix.upCell;
        windowTipping[nextSlot] = fix.tipping;
        windowHasPosition[nextSlot] = positionUsable;
        if (positionUsable) {
            windowOffsetX[nextSlot] = fix.robot.getX() - odometryAtFrame.getX();
            windowOffsetY[nextSlot] = fix.robot.getY() - odometryAtFrame.getY();
        }
        windowNanos[nextSlot] = nowNanos;
        nextSlot = (nextSlot + 1) % AprilTagConfig.HIVE_WINDOW_FRAMES;
        framesInWindow = Math.min(framesInWindow + 1, AprilTagConfig.HIVE_WINDOW_FRAMES);
    }

    private void clearWindow() {
        framesInWindow = 0;
        nextSlot = 0;
        lastFrameId = -1;
    }

    /** True if slot i is recent enough to count (a frame from before the robot looked away does not). */
    private boolean isFresh(int i, long nowNanos) {
        return (nowNanos - windowNanos[i]) / 1e6 <= AprilTagConfig.HIVE_WINDOW_MAX_AGE_MS;
    }

    /** The CELL that wins the window, or null if none wins. */
    HiveTagLocalizer.Cell windowUpCell(long nowNanos) {
        int framesSayA = 0;
        int framesSayB = 0;
        for (int i = 0; i < framesInWindow; i++) {
            if (!isFresh(i, nowNanos)) continue;
            if (windowUpCell[i] == HiveTagLocalizer.Cell.A) framesSayA++;
            if (windowUpCell[i] == HiveTagLocalizer.Cell.B) framesSayB++;
        }
        if (framesSayA >= AprilTagConfig.HIVE_WINDOW_MIN_AGREE && framesSayB == 0) return HiveTagLocalizer.Cell.A;
        if (framesSayB >= AprilTagConfig.HIVE_WINDOW_MIN_AGREE && framesSayA == 0) return HiveTagLocalizer.Cell.B;
        return null;
    }

    /** True if enough recent frames say the HIVE is tipping. */
    boolean windowTipping(long nowNanos) {
        int framesSayTipping = 0;
        for (int i = 0; i < framesInWindow; i++) {
            if (isFresh(i, nowNanos) && windowTipping[i]) framesSayTipping++;
        }
        return framesSayTipping >= AprilTagConfig.HIVE_WINDOW_MIN_TIPPING;
    }

    /** Odometry now plus the average recent offset, or null with fewer than HIVE_WINDOW_MIN_FIXES recent fixes. */
    Position windowPosition(long nowNanos, Position odometryNow) {
        int count = 0;
        double sumX = 0;
        double sumY = 0;
        for (int i = 0; i < framesInWindow; i++) {
            if (!isFresh(i, nowNanos) || !windowHasPosition[i]) continue;
            sumX += windowOffsetX[i];
            sumY += windowOffsetY[i];
            count++;
        }
        if (count < AprilTagConfig.HIVE_WINDOW_MIN_FIXES) return null;
        return new Position(odometryNow.getX() + sumX / count, odometryNow.getY() + sumY / count, odometryNow.getTheta());
    }

    private void publishGoalState(long nowNanos) {
        HiveTagLocalizer.Cell winner = windowUpCell(nowNanos);
        boolean tipping = windowTipping(nowNanos);

        // The HIVE only changes on a TIP, so the last winner stands until a new one wins.
        HiveTagLocalizer.Cell previous = SharedData.getHiveUpCell();
        if (winner != null) SharedData.setHiveUpCell(winner);
        boolean wasTipping = SharedData.isHiveTipping();
        SharedData.setHiveTipping(tipping);

        boolean changed = (winner != null && winner != previous) || tipping != wasTipping;
        if (changed) {
            HiveTagLocalizer.Cell now = winner != null ? winner : previous;
            KLog.d(TAG, () -> "event=goalState upCell=" + now + " was=" + previous + " tipping=" + tipping);
        }
    }

    private void publishPosition(long nowNanos, HiveTagLocalizer.Fix frameFix, Position odometryAtFrame) {
        Position odometryNow = SharedData.getOdometryWheelIMUPosition();
        Position fix = windowPosition(nowNanos, odometryNow);
        if (fix == null) return;
        double offX = fix.getX() - odometryNow.getX();
        double offY = fix.getY() - odometryNow.getY();
        SharedData.setLimelightFix(fix, offX, offY);
        if (KLog.isDebug()) {
            // Frame vs odometry side by side, to find why the fix error changes with heading / turret angle.
            KLog.d(TAG, () -> String.format(Locale.US,
                    "event=fix x=%.0f y=%.0f offX=%.0f offY=%.0f frameX=%.0f frameY=%.0f odoX=%.0f odoY=%.0f odoDeg=%.1f turretDeg=%.1f tags=%d",
                    fix.getX(), fix.getY(), offX, offY, frameFix.robot.getX(), frameFix.robot.getY(),
                    odometryAtFrame.getX(), odometryAtFrame.getY(), Math.toDegrees(odometryAtFrame.getTheta()),
                    Math.toDegrees(turret.getAngleRad()), frameFix.tagsUsed));
        }
    }
}
