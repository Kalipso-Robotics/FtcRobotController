package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblobbing.BallColorBlobProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

/**
 * The Arducam ball camera: portal + pollen/nectar detector + raytracer, wired the way PollenNectarLive proved.
 * Every frame the raytracer publishes SharedData.setBallInformation, so autos (and VisionPickupAction) just read SharedData.
 * Usage: BallVision ballVision = BallVision.start(hardwareMap, BallVision.Detector.TFLITE, false); ... ballVision.close();
 */
public class BallVision {
    // Must match the webcam name in the robot configuration
    private static final String WEBCAM_NAME = "Arducam";

    /** What finds the balls in each frame. Both feed the same raytracer, so nothing downstream changes. */
    public enum Detector {
        /** The trained model: pollen and nectar, robust to lighting. */
        TFLITE,
        /** HSV color thresholds: no model, cheaper, but needs its ranges tuned for the venue lighting. */
        COLOR_BLOB
    }

    public final KVisionProcessor<List<VisionRecognition>> detector;
    public final Raytracer raytracer;
    public final VisionPortal portal;

    private BallVision(KVisionProcessor<List<VisionRecognition>> detector, Raytracer raytracer, VisionPortal portal) {
        this.detector = detector;
        this.raytracer = raytracer;
        this.portal = portal;
    }

    /** The TFLite detector. liveView: the Driver Station camera preview. Off for matches (it costs CPU). */
    public static BallVision start(HardwareMap hardwareMap, boolean liveView) {
        return start(hardwareMap, Detector.TFLITE, liveView);
    }

    public static BallVision start(HardwareMap hardwareMap, Detector which, boolean liveView) {
        KVisionProcessor<List<VisionRecognition>> detector = which == Detector.COLOR_BLOB
                ? new BallColorBlobProcessor()
                // 0.35 confidence cutoff; NMS is on by default inside the detector
                : new TFLitePollenNectarDetector(hardwareMap.appContext).withMinConfidence(0.35f);

        // Raytraces every frame the detector produces: turns each pixel box into a ball position.
        // It reads the camera size/lens/mount from VisionConfig.ARDUCAM, so the portal below must
        // stay 1280x720 or the positions will be wrong.
        Raytracer raytracer = new Raytracer(detector, VisionConfig.ARDUCAM);

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(detector)
                .enableLiveView(liveView)
                .setAutoStopLiveView(false)
                .build();
        return new BallVision(detector, raytracer, portal);
    }

    public void close() {
        portal.close();
    }
}
