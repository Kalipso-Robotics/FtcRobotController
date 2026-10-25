package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;
import java.util.List;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.BallInformation;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

@TeleOp(name = "Pollen Nectar Live", group = "test")
public class PollenNectarLive extends LinearOpMode {

    // Must match the webcam name in the robot configuration
    private static final String WEBCAM_NAME = "Arducam";

    @Override
    public void runOpMode() {
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
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();

        while (opModeInInit() || opModeIsActive()) {
            telemetry.addData("Camera", portal.getCameraState());
            telemetry.addData("FPS", "%.1f", portal.getFps());
            telemetry.addData("Model", detector.getDiagnosticSummary());

            // The raytracer's last frame. The three lists line up: index i is the same ball in each.
            // Boxes it can't raytrace (touching the frame edge, impossible ray) are left out,
            // so this can be shorter than the detector's raw list.
            Raytracer.Result result = raytracer.getLastResult();
            List<VisionRecognition> raw = detector.getLatestResult();
            telemetry.addData("Raw boxes (detector)", raw == null ? 0 : raw.size());
            telemetry.addData("Raytraced (kept)", result.detections.size());

            // One line every frame, even with no balls, so logcat shows the opmode is running.
            final int rawCount = raw == null ? 0 : raw.size();
            final int keptCount = result.detections.size();
            KLog.d("PollenNectarLive", () -> String.format(Locale.US,
                    "frame: raw=%d kept=%d", rawCount, keptCount));
            // Every raw box, so boxes the raytracer drops still show their corners.
            if (raw != null) {
                for (VisionRecognition r : raw) {
                    KLog.d("PollenNectarLive", () -> String.format(Locale.US,
                            "raw: %s conf=%.2f botLeft=(%.0f,%.0f) topRight=(%.0f,%.0f)",
                            r.label, r.confidence, r.left, r.bottom, r.right, r.top));
                }
            }
            for (int i = 0; i < result.detections.size(); i++) {
                VisionRecognition box = result.detections.get(i);
                Raytracer.Estimate est = result.estimates.get(i);
                BallInformation ball = result.balls.get(i);

                // Pixel box corners. Pixel (0,0) is the top-left of the image, so bottom-left is
                // (left, bottom) and top-right is (right, top).
                telemetry.addLine(String.format(Locale.US, "%s %.2f botLeft=(%.0f,%.0f) topRight=(%.0f,%.0f)",
                        box.label, box.confidence, box.left, box.bottom, box.right, box.top));

                // Ball relative to the robot in mm (left / forward). This is the number to trust here.
                // Field x,y uses the odometry pose, which this opmode doesn't update, so it is only
                // meaningful when odometry is running.
                telemetry.addLine(String.format(Locale.US,
                        "   robot: left=%.0f fwd=%.0f mm | field: x=%.0f y=%.0f | depth ok=%b",
                        est.robotPos.getX(), est.robotPos.getY(), ball.x, ball.y,
                        est.isConsistent()));

                // Same thing to logcat (filter on "KLog_PollenNectarLive"), plus the raytracing details.
                final int index = i;
                KLog.d("PollenNectarLive", () -> String.format(Locale.US,
                        "ball %d: %s conf=%.2f | box botLeft=(%.0f,%.0f) topRight=(%.0f,%.0f)"
                                + " | robot left=%.0f fwd=%.0f mm | field x=%.0f y=%.0f"
                                + " | rayDepth=%.0f sizeDepthV=%.0f sizeDepthH=%.0f mm"
                                + " | psi=%.1f theta=%.1f deg | depth ok=%b",
                        index, box.label, box.confidence,
                        box.left, box.bottom, box.right, box.top,
                        est.robotPos.getX(), est.robotPos.getY(), ball.x, ball.y,
                        est.rayDepthMM, est.sizeDepthVMM, est.sizeDepthHMM,
                        Math.toDegrees(est.psiRad), Math.toDegrees(est.thetaRad),
                        est.isConsistent()));
            }
            telemetry.update();
            sleep(50);
        }

        portal.close();
    }
}