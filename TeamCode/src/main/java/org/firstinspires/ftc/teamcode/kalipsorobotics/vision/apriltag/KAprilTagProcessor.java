package org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag;

import android.graphics.Canvas;
import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.CameraIntrinsics;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * AprilTag detection for any webcam on the shared VisionPortal.
 *
 * WHY THIS EXISTS:
 *   Until now AprilTags only came from the Limelight, which is its own camera with its
 *   own coprocessor. This class puts AprilTags on the SAME camera frame as the color
 *   blob processors, so one Arducam can answer "where am I" and "where are the balls"
 *   at once. It is a VisionProcessor, so VisionManager registers it exactly like
 *   ArtifactColorBlobDetectionProcessor, and it is an AprilTagCamera, so
 *   AprilTagDetectionAction consumes it exactly like LimelightAprilTagCamera.
 *
 * ONE CAMERA, TWO CONSUMERS:
 *   ArtifactColorBlobDetectionProcessor artifacts = new ArtifactColorBlobDetectionProcessor();
 *   KAprilTagProcessor tags = new KAprilTagProcessor.Builder().build();
 *
 *   VisionManager camera = new VisionManager.Builder(hardwareMap)
 *           .addProcessor(artifacts)
 *           .addProcessor(tags)
 *           .streamImmediately()
 *           .build();
 *
 *   // Relocalization - the action does the transform chain, this just supplies tags.
 *   AprilTagDetectionAction relocalize = new AprilTagDetectionAction(
 *           opModeUtilities, turret, AprilTagConfig.getGoalAprilTagId(alliance), alliance, tags);
 *
 *   // Anything else that wants blobs reads them straight off the processor.
 *   DetectedBlob ball = artifacts.getLargestPurpleBlob();
 *
 * WHY IT DELEGATES INSTEAD OF EXTENDING KVisionProcessor:
 *   KVisionProcessor owns processFrame() and hands subclasses only the Mat. The SDK's
 *   AprilTag detector needs the frame's capture timestamp as well, and it draws its own
 *   overlay from its own detection objects. Wrapping the SDK processor keeps both of
 *   those intact. The public read side (getLatestResult / hasResult /
 *   getDiagnosticSummary) is named to match KVisionProcessor so callers see one API.
 *
 * THREADING:
 *   processFrame() and onDrawFrame() run on the VisionPortal camera thread. Everything
 *   the robot thread reads goes through a volatile reference, so getLatestObservations()
 *   never blocks the main loop.
 */
public class KAprilTagProcessor implements VisionProcessor, AprilTagCamera {

    private static final String TAG = "KAprilTagProcessor";

    private final AprilTagProcessor delegate;
    private final CameraIntrinsics intrinsics;
    private final float decimation;

    /** Latest converted detections. Written on the camera thread, read on the robot thread. */
    private volatile List<TagObservation> latestObservations = Collections.emptyList();

    /** Flipped by stop()/start(). See the note on stop() - this gates reads, not CPU. */
    private volatile boolean running = true;

    private volatile int frameCount = 0;
    private volatile int lastDetectionCount = 0;
    private volatile int lastPoseSolveFailures = 0;

    private KAprilTagProcessor(AprilTagProcessor delegate, CameraIntrinsics intrinsics, float decimation) {
        this.delegate = delegate;
        this.intrinsics = intrinsics;
        this.decimation = decimation;
    }

    // -------------------------------------------------------------------------
    // AprilTagCamera - what AprilTagDetectionAction consumes
    // -------------------------------------------------------------------------

    /**
     * No-op: the VisionPortal is already streaming by the time this processor exists.
     * Present so AprilTagDetectionAction can call it uniformly across camera backends.
     */
    @Override
    public void start() {
        running = true;
    }

    /**
     * Stops handing out observations. This does NOT stop the detector - to actually
     * reclaim the CPU, call {@code visionManager.disable(processor)}, which unhooks it
     * from the portal. Kept cheap on purpose so an action can gate itself without
     * disturbing anything else reading the same camera.
     */
    @Override
    public void stop() {
        running = false;
    }

    @Override
    public List<TagObservation> getLatestObservations() {
        return running ? latestObservations : Collections.<TagObservation>emptyList();
    }

    // -------------------------------------------------------------------------
    // Read side - mirrors the KVisionProcessor API so all processors look alike
    // -------------------------------------------------------------------------

    /** Latest observations, never null. Empty until the first tag is seen. */
    public List<TagObservation> getLatestResult() {
        return getLatestObservations();
    }

    /** True when at least one tag was visible in the most recent processed frame. */
    public boolean hasResult() {
        return !getLatestObservations().isEmpty();
    }

    /** The observation for one tag ID, or null if that tag is not currently visible. */
    public TagObservation getObservation(int tagId) {
        for (TagObservation observation : getLatestObservations()) {
            if (observation.getTagId() == tagId) return observation;
        }
        return null;
    }

    public boolean isTagVisible(int tagId) {
        return getObservation(tagId) != null;
    }

    /**
     * Raw SDK detections for the latest frame - pixel corners, decision margin, ftcPose,
     * and the SDK's own robotPose. Use this only for things TagObservation deliberately
     * drops (pixel-space work, detection quality checks); localization should go through
     * getLatestObservations() so it stays camera-agnostic.
     */
    public List<AprilTagDetection> getRawDetections() {
        List<AprilTagDetection> detections = delegate.getDetections();
        return (detections == null) ? Collections.<AprilTagDetection>emptyList() : detections;
    }

    public String getDiagnosticSummary() {
        return String.format(Locale.US,
                "frames=%d tags=%d poseFailures=%d decimation=%.1f solveMs=%d",
                frameCount, lastDetectionCount, lastPoseSolveFailures, decimation,
                delegate.getPerTagAvgPoseSolveTime());
    }

    public int getFrameCount()        { return frameCount; }
    public int getLastDetectionCount(){ return lastDetectionCount; }

    /** The wrapped SDK processor, for runtime knobs this class does not surface. */
    public AprilTagProcessor getDelegate() { return delegate; }

    /**
     * Lower decimation (1) sees small/far tags but costs more CPU; higher (3) is fast but
     * only finds large/near tags. Safe to call mid-OpMode.
     */
    public void setDecimation(float decimation) {
        delegate.setDecimation(decimation);
    }

    // -------------------------------------------------------------------------
    // VisionProcessor - straight delegation, plus the conversion to TagObservation
    // -------------------------------------------------------------------------

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        if (intrinsics != null
                && (width != CameraIntrinsics.CAM_WIDTH || height != CameraIntrinsics.CAM_HEIGHT)) {
            Log.w(TAG, String.format(Locale.US,
                    "Stream is %dx%d but the supplied CameraIntrinsics are calibrated for %dx%d. "
                            + "Pose estimates will be wrong - recalibrate or drop withIntrinsics().",
                    width, height, CameraIntrinsics.CAM_WIDTH, CameraIntrinsics.CAM_HEIGHT));
        }
        delegate.init(width, height, calibration);
        delegate.setDecimation(decimation);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Object detectionContext = delegate.processFrame(frame, captureTimeNanos);
        frameCount++;
        latestObservations = convert(detectionContext);
        // Handed back untouched so onDrawFrame() gets exactly what the SDK annotator wants.
        return detectionContext;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        delegate.onDrawFrame(canvas, onscreenWidth, onscreenHeight,
                scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
    }

    /**
     * Turns the SDK's detections into the normalized TagObservations the rest of the
     * robot speaks. Detections whose tag is missing from the tag library carry no pose
     * (the solver needs the tag's physical size) and are skipped.
     */
    @SuppressWarnings("unchecked")
    private List<TagObservation> convert(Object detectionContext) {
        if (!(detectionContext instanceof List)) {
            lastDetectionCount = 0;
            return Collections.emptyList();
        }

        List<AprilTagDetection> detections = (List<AprilTagDetection>) detectionContext;
        lastDetectionCount = detections.size();
        if (detections.isEmpty()) {
            lastPoseSolveFailures = 0;
            return Collections.emptyList();
        }

        int poseFailures = 0;
        List<TagObservation> observations = new ArrayList<>(detections.size());
        for (AprilTagDetection detection : detections) {
            if (detection.rawPose == null || detection.rawPose.R == null) {
                poseFailures++;
                continue;
            }
            // Output units are set to MM on the builder, so rawPose is already in mm and
            // already in the OpenCV camera/tag frames fromOpenCvPose() documents.
            observations.add(TagObservation.fromOpenCvPose(
                    detection.id,
                    detection.rawPose.x,
                    detection.rawPose.y,
                    detection.rawPose.z,
                    detection.rawPose.R));
        }
        lastPoseSolveFailures = poseFailures;
        return observations;
    }

    // -------------------------------------------------------------------------
    // Builder
    // -------------------------------------------------------------------------

    public static class Builder {

        private CameraIntrinsics intrinsics = CameraIntrinsics.ARDUCAM;
        private AprilTagLibrary tagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();
        private AprilTagProcessor.TagFamily tagFamily = AprilTagProcessor.TagFamily.TAG_36h11;
        private float decimation = 2f;
        private int threads = AprilTagProcessor.THREADS_DEFAULT;
        private boolean drawAxes = true;
        private boolean drawCube = false;
        private boolean drawOutline = true;
        private boolean drawTagId = true;

        /**
         * Lens calibration used by the pose solver. Defaults to CameraIntrinsics.ARDUCAM,
         * which is calibrated for 640x480 - build the portal at that resolution or supply
         * intrinsics that match it.
         *
         * Pass null to let the SDK use its own built-in calibration for the camera, which
         * is the right move for a webcam this team has not calibrated.
         */
        public Builder withIntrinsics(CameraIntrinsics intrinsics) {
            this.intrinsics = intrinsics;
            return this;
        }

        /**
         * Which tags exist and how big they are. Defaults to the current season's library
         * (AprilTagGameDatabase), which is what makes pose estimation possible at all -
         * a tag missing from the library gets detected but never gets a pose.
         */
        public Builder withTagLibrary(AprilTagLibrary tagLibrary) {
            this.tagLibrary = tagLibrary;
            return this;
        }

        public Builder withTagFamily(AprilTagProcessor.TagFamily tagFamily) {
            this.tagFamily = tagFamily;
            return this;
        }

        /**
         * Detector downscaling. 1 finds small/far tags at full CPU cost, 2 is the balanced
         * default, 3 is cheap but near-tags-only. Applied on init and changeable later
         * with setDecimation().
         */
        public Builder withDecimation(float decimation) {
            this.decimation = decimation;
            return this;
        }

        /** Detector worker threads. Lower this when sharing the CPU with heavy processors. */
        public Builder withThreads(int threads) {
            this.threads = threads;
            return this;
        }

        /** Driver Station overlay. Turn it all off to save a little draw time. */
        public Builder withDrawing(boolean axes, boolean cube, boolean outline, boolean tagId) {
            this.drawAxes = axes;
            this.drawCube = cube;
            this.drawOutline = outline;
            this.drawTagId = tagId;
            return this;
        }

        public KAprilTagProcessor build() {
            AprilTagProcessor.Builder builder = new AprilTagProcessor.Builder()
                    .setTagLibrary(tagLibrary)
                    .setTagFamily(tagFamily)
                    // MM/RADIANS keeps the SDK's numbers on the same scale as everything
                    // else on this robot, so no unit conversion leaks into the conversion.
                    .setOutputUnits(DistanceUnit.MM, AngleUnit.RADIANS)
                    .setNumThreads(threads)
                    .setDrawAxes(drawAxes)
                    .setDrawCubeProjection(drawCube)
                    .setDrawTagOutline(drawOutline)
                    .setDrawTagID(drawTagId);

            if (intrinsics != null) {
                builder.setLensIntrinsics(intrinsics.getFx(), intrinsics.getFy(),
                        intrinsics.getCx(), intrinsics.getCy());
            }

            return new KAprilTagProcessor(builder.build(), intrinsics, decimation);
        }
    }
}
