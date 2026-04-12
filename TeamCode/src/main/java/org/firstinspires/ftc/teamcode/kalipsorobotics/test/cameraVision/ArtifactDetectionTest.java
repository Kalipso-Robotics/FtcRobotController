package org.firstinspires.ftc.teamcode.kalipsorobotics.test.cameraVision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.ArtifactDetectionProcessor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.DetectedBlob;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionManager;

/**
 * Tuning OpMode for ArtifactDetectionProcessor.
 *
 * Demonstrates how VisionManager lets multiple processors share one camera:
 * AprilTags and color blobs both run simultaneously from the same frame.
 *
 * Connect HDMI or use scrcpy to see the annotated live stream.
 * Runs during INIT so you can preview and tune before pressing start.
 *
 * Gamepad controls:
 *   [X] — preview purple blobs only (use to tune PURPLE_HSV thresholds)
 *   [B] — preview green blobs only  (use to tune GREEN_HSV thresholds)
 *   [A] — preview both colors
 *
 * Tune these constants in ArtifactDetectionProcessor / KColorBlobProcessor
 * until detection is solid across the full expected detection range:
 *   PURPLE_HSV_LOWER / UPPER  — hue 117-155, raise S_min if walls false-trigger
 *   GREEN_HSV_LOWER  / UPPER  — hue 40-85,   narrow if floor causes false-positives
 *   minContourArea            — raise if noise blobs appear
 *   minCircularity            — lower (e.g. 0.4) if occluded balls are missed
 */
@Disabled
@TeleOp(name = "Test: Artifact Detection", group = "Test Vision")
public class ArtifactDetectionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // --- Setup ---
        ArtifactDetectionProcessor artifacts = new ArtifactDetectionProcessor();
        AprilTagProcessor aprilTags = AprilTagProcessor.easyCreateWithDefaults();

        VisionManager visionManager = new VisionManager.Builder(hardwareMap)
                .addProcessor(artifacts)
                .addProcessor(aprilTags)
                .build();

        telemetry.setMsTransmissionInterval(50);

        // --- Loop (runs in INIT and ACTIVE) ---
        while (opModeIsActive() || opModeInInit()) {

            // Live color-mode switching via gamepad for threshold tuning
            if (gamepad1.x) visionManager.disable(artifacts);  // disable to compare
            else if (gamepad1.b) { visionManager.enable(artifacts); }
            else if (gamepad1.a) { visionManager.enable(artifacts); }

            DetectedBlob largestPurple = artifacts.getLargestPurpleBlob();
            DetectedBlob largestGreen  = artifacts.getLargestGreenBlob();

            telemetry.addLine("=== Artifact Detection ===");
            telemetry.addData("April Tags detected", aprilTags.getDetections().size());
            telemetry.addData("Has purple", artifacts.hasPurpleBlob());
            telemetry.addData("Has green",  artifacts.hasGreenBlob());

            if (largestPurple != null) telemetry.addLine("PURPLE → " + largestPurple);
            if (largestGreen  != null) telemetry.addLine("GREEN  → " + largestGreen);

            telemetry.addLine("[X]=disable artifacts  [B/A]=enable");
            telemetry.update();
            sleep(50);
        }

        visionManager.close();
    }
}
