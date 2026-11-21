package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

/**
 * The Arducam ball camera: portal + pollen/nectar detector + raytracer, wired the way PollenNectarLive proved.
 * Every frame the raytracer publishes SharedData.setBallInformation, so autos just read SharedData.
 * Usage: BallVision ballVision = BallVision.start(hardwareMap, false); ... ballVision.close();
 */
public class BallVision {
    // Must match the webcam name in the robot configuration
    private static final String WEBCAM_NAME = "Arducam";

    public final TFLitePollenNectarDetector detector;
    public final Raytracer raytracer;
    public final VisionPortal portal;

    private BallVision(TFLitePollenNectarDetector detector, Raytracer raytracer, VisionPortal portal) {
        this.detector = detector;
        this.raytracer = raytracer;
        this.portal = portal;
    }

    /** liveView: the Driver Station camera preview. Off for matches (it costs CPU). */
    public static BallVision start(HardwareMap hardwareMap, boolean liveView) {
        // 0.35 confidence cutoff; NMS is on by default inside the detector
        TFLitePollenNectarDetector detector = new TFLitePollenNectarDetector(hardwareMap.appContext)
                .withMinConfidence(0.35f);

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
