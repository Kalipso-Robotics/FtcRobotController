package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@TeleOp(name = "Pollen Nectar Stream Test", group = "test")
public class PollenNectarStreamTest extends LinearOpMode {

    // Must match the webcam name in the robot configuration
    private static final String WEBCAM_NAME = "Arducam";

    @Override
    public void runOpMode() {
        SharedData.resetPollenNectarDetections();
        // 0.35 confidence cutoff; NMS is on by default inside the detector
        TFLitePollenNectarDetector detector = new TFLitePollenNectarDetector(hardwareMap.appContext)
                .withMinConfidence(0.35f);

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
            telemetry.addData("Age ms", System.currentTimeMillis() - SharedData.getPollenNectarDetectionsTimeMs());
            List<BallDetection> recs = SharedData.getPollenNectarDetections();
            KLog.d("PollenNectar",()->"count="+recs.size());
            telemetry.addData("Detections", recs == null ? 0 : recs.size());
            if (recs != null) {
                for (BallDetection r : recs) {
                    telemetry.addLine(String.format("%s %.2f [%.0f,%.0f -> %.0f,%.0f]",
                            r.label, r.confidence, r.left, r.bottom, r.right, r.top));
                    KLog.d("PollenNectar", () -> String.format("%s %.2f [%.0f,%.0f -> %.0f,%.0f]",
                            r.label, r.confidence, r.left, r.bottom, r.right,r.top));
                }
            }
            telemetry.update();
            sleep(50);
        }

        portal.close();
    }
}