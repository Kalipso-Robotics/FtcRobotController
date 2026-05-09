package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Owns the robot's single VisionPortal and manages all processors on it.
 *
 * WHY ONE PORTAL:
 *   Each VisionPortal starts a dedicated OS thread for camera I/O. Creating
 *   more than one portal on the same camera either crashes or silently steals
 *   frames from the other. VisionManager enforces one portal so all processors
 *   share the same frame at zero extra CPU cost.
 *
 * MULTIPLE CONSUMERS, SAME FRAME:
 *   Any number of processors can be registered. VisionPortal delivers every
 *   frame to every processor on the same camera thread. Each processor runs
 *   independently — one team member's auto code can read artifact blobs while
 *   another reads AprilTags from the same camera without any conflicts.
 *
 * USAGE:
 *   ArtifactDetectionProcessor artifacts = new ArtifactDetectionProcessor();
 *   AprilTagProcessor aprilTags = AprilTagProcessor.easyCreateWithDefaults();
 *
 *   VisionManager visionManager = new VisionManager.Builder(hardwareMap)
 *       .addProcessor(artifacts)
 *       .addProcessor(aprilTags)
 *       .build();
 *
 *   // Enable / disable at runtime — disabled processors cost zero CPU
 *   visionManager.enable(artifacts);
 *   visionManager.disable(artifacts);
 *
 *   // Pause / resume the entire camera (saves battery mid-auto)
 *   visionManager.pauseCamera();
 *   visionManager.resumeCamera();
 *
 *   // In OpMode.stop()
 *   visionManager.close();
 */
public class VisionManager {

    private final VisionPortal portal;

    private VisionManager(VisionPortal portal) {
        this.portal = portal;
    }

    // -------------------------------------------------------------------------
    // Runtime controls
    // -------------------------------------------------------------------------

    /** Resume a processor that was previously disabled. */
    public void enable(VisionProcessor processor) {
        portal.setProcessorEnabled(processor, true);
    }

    /** Disable a processor — it will no longer receive frames or consume CPU. */
    public void disable(VisionProcessor processor) {
        portal.setProcessorEnabled(processor, false);
    }

    /**
     * Pause the camera stream entirely.
     * Useful mid-autonomous when no vision data is needed for several seconds.
     * Note: resuming takes ~200ms while the camera reinitialises.
     */
    public void pauseCamera()  { portal.stopStreaming();  }
    public void resumeCamera() { portal.resumeStreaming(); }

    /**
     * Lock exposure and gain to fixed values for consistent color detection.
     *
     * Call this once after build(), before your autonomous loop starts. Locking
     * prevents the camera from auto-adjusting to venue lighting changes, which
     * would shift the HSV values you tuned with artifact_tuner.py.
     *
     * Use the same exposureMs and gain values you set in artifact_tuner.py so the
     * HSV thresholds transfer correctly from your laptop to the robot.
     *
     * Recommended starting values for the Arducam:
     *   exposureMs = 20,  gain = 250  (bright venue)
     *   exposureMs = 35,  gain = 350  (dim venue)
     *
     * @param exposureMs  Shutter time in milliseconds. Lower = darker but less motion blur.
     * @param gain        Sensor gain. Higher = brighter but more noise.
     * @return true if controls were applied, false if camera did not start in time.
     */
    public boolean lockCameraControls(long exposureMs, int gain) {
        long deadline = System.currentTimeMillis() + 3_000;
        while (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            if (System.currentTimeMillis() > deadline) return false;
            try { Thread.sleep(20); } catch (InterruptedException ignored) {}
        }

        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        if (exposure != null) {
            exposure.setMode(ExposureControl.Mode.Manual);
            exposure.setExposure(exposureMs, TimeUnit.MILLISECONDS);
        }

        GainControl gainControl = portal.getCameraControl(GainControl.class);
        if (gainControl != null) {
            gainControl.setGain(gain);
        }

        return true;
    }

    public VisionPortal getPortal() { return portal; }

    public void close() {
        if (portal != null) portal.close();
    }

    // -------------------------------------------------------------------------
    // Builder
    // -------------------------------------------------------------------------

    public static class Builder {

        private final HardwareMap hardwareMap;
        private String cameraName = "Arducam";
        private Size resolution = new Size(640, 480);
        private VisionPortal.StreamFormat streamFormat = VisionPortal.StreamFormat.MJPEG;
        private boolean streamImmediately = false;
        private final List<VisionProcessor> processors = new ArrayList<>();

        public Builder(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
        }

        /** Override the hardware map camera name. Default: "Arducam". */
        public Builder withCamera(String cameraName) {
            this.cameraName = cameraName;
            return this;
        }

        /**
         * Override the capture resolution. Default: 640x480.
         * Supported resolutions for this camera: 320x240, 640x480, 800x600, 1280x720, 1280x800.
         */
        public Builder withResolution(int width, int height) {
            this.resolution = new Size(width, height);
            return this;
        }

        /**
         * Override the stream format. Default: MJPEG.
         * MJPEG is preferred — lower USB bandwidth and better color fidelity than YUY2.
         * Use YUY2 only if you need raw uncompressed frames.
         */
        public Builder withStreamFormat(VisionPortal.StreamFormat format) {
            this.streamFormat = format;
            return this;
        }

        /**
         * Start the camera stream immediately on build() instead of waiting for
         * an explicit resumeCamera() call. Adds ~500ms to build() but means
         * frames are available the moment the object is constructed.
         *
         * Default: false — build() returns instantly; call visionManager.resumeCamera()
         * when you actually need frames (e.g. at opModeStart).
         */
        public Builder streamImmediately() {
            this.streamImmediately = true;
            return this;
        }

        /**
         * Register any VisionProcessor — KVisionProcessor subclasses,
         * KRoboflowDetector subclasses, AprilTagProcessor, or any custom processor.
         * All registered processors share the same camera frame.
         */
        public Builder addProcessor(VisionProcessor processor) {
            processors.add(processor);
            return this;
        }

        public VisionManager build() {
            VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                    .setCameraResolution(resolution)
                    .setStreamFormat(streamFormat)
                    .setAutoStartStreamOnBuild(streamImmediately);

            for (VisionProcessor processor : processors) {
                portalBuilder.addProcessor(processor);
            }

            return new VisionManager(portalBuilder.build());
        }
    }
}
