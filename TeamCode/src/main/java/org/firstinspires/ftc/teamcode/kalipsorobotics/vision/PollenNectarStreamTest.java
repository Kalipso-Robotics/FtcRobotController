package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.BallInformation;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@TeleOp(name = "Pollen Nectar Stream Test", group = "test")
public class PollenNectarStreamTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        SharedData.resetPollenNectarDetections();
        SharedData.resetBallInformation();
        // 0.35 confidence cutoff; NMS is on by default inside the detector
        TFLitePollenNectarDetector detector = new TFLitePollenNectarDetector(hardwareMap.appContext)
                .withMinConfidence(0.35f);

        // Raytraces every frame on the camera thread and publishes to SharedData.
        // Field frame == robot frame here: no odometry runs, the robot sits at the origin.
        new Raytracer(detector, VisionConfig.ARDUCAM);

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, VisionConfig.ARDUCAM.name))
                .setCameraResolution(new Size(VisionConfig.ARDUCAM.width, VisionConfig.ARDUCAM.height))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(detector)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();

        while (opModeInInit() || opModeIsActive()) {
            SharedData.setOdometryWheelIMUPosition(new Position(0, 0, 0));
            telemetry.addData("Camera", portal.getCameraState());
            telemetry.addData("FPS", "%.1f", portal.getFps());
            telemetry.addData("Model", detector.getDiagnosticSummary());
            telemetry.addData("Age ms", System.currentTimeMillis() - SharedData.getPollenNectarDetectionsTimeMs());
            List<VisionRecognition> recs = SharedData.getPollenNectarDetections();
            KLog.d("PollenNectar",()->"count="+recs.size());
            telemetry.addData("Detections", recs == null ? 0 : recs.size());
            if (recs != null) {
                for (VisionRecognition r : recs) {
                    telemetry.addLine(String.format("%s %.2f [%.0f,%.0f -> %.0f,%.0f]",
                            r.label, r.confidence, r.left, r.bottom, r.right, r.top));
                    KLog.d("PollenNectar", () -> String.format("%s %.2f [%.0f,%.0f -> %.0f,%.0f]",
                            r.label, r.confidence, r.left, r.bottom, r.right,r.top));
                }
            }
            List<BallInformation> balls = SharedData.getBallInformation();
            telemetry.addData("Balls raytraced", "%d (age %d ms)", balls.size(),
                    System.currentTimeMillis() - SharedData.getBallInformationTimeMs());
            for (BallInformation b : balls) {
                telemetry.addLine(String.format("%s %.0f%% x=%.0f y=%.0f dist=%.0f mm",
                        b.type, b.confidence * 100, b.x, b.y, b.distanceMM));
                KLog.d("PollenNectar", () -> "ball " + b + " dist=" + b.distanceMM);
            }
            telemetry.update();
            sleep(50);
        }

        portal.close();
    }
}