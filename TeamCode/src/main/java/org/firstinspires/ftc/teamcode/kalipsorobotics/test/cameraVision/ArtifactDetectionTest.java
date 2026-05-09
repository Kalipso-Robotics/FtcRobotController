package org.firstinspires.ftc.teamcode.kalipsorobotics.test.cameraVision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.ArtifactDetectionProcessor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.DetectedBlob;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionManager;

/**
 * Live tuning OpMode for ArtifactDetectionProcessor.
 *
 * HOW TO TUNE WITHOUT REDEPLOYING:
 *   1. Connect your phone/laptop to the robot's WiFi (192.168.43.1)
 *   2. Open http://192.168.43.1:8080 in a browser → FTC Dashboard
 *   3. Expand "ArtifactDetectionTest" in the variable panel on the right
 *   4. Adjust EXPOSURE_MS and GAIN — camera updates instantly
 *   5. When detection looks solid, note the values and paste them into
 *      your auto OpMode's lockCameraControls() call.
 *
 * Gamepad:
 *   [X] — disable artifact processor (see raw camera feed)
 *   [A] — re-enable artifact processor
 */
@Config
@Disabled
@TeleOp(name = "Test: Artifact Detection", group = "Test Vision")
public class ArtifactDetectionTest extends LinearOpMode {

    // ── Tunable via FTC Dashboard — change without redeploying ───────────────
    public static long EXPOSURE_MS = 20;
    public static int  GAIN        = 250;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        ArtifactDetectionProcessor artifacts = new ArtifactDetectionProcessor();

        VisionManager visionManager = new VisionManager.Builder(hardwareMap)
                .addProcessor(artifacts)
                .streamImmediately()
                .build();

        visionManager.lockCameraControls(EXPOSURE_MS, GAIN);

        long prevExposure = EXPOSURE_MS;
        int  prevGain     = GAIN;

        while (opModeIsActive() || opModeInInit()) {

            // Re-apply camera controls whenever dashboard values change
            if (EXPOSURE_MS != prevExposure || GAIN != prevGain) {
                visionManager.lockCameraControls(EXPOSURE_MS, GAIN);
                prevExposure = EXPOSURE_MS;
                prevGain     = GAIN;
            }

            if (gamepad1.x) visionManager.disable(artifacts);
            if (gamepad1.a) visionManager.enable(artifacts);

            DetectedBlob largestPurple = artifacts.getLargestPurpleBlob();
            DetectedBlob largestGreen  = artifacts.getLargestGreenBlob();

            telemetry.addData("EXPOSURE_MS", EXPOSURE_MS);
            telemetry.addData("GAIN",        GAIN);
            telemetry.addLine("───────────────────");
            telemetry.addData("Has purple", artifacts.hasPurpleBlob());
            telemetry.addData("Has green",  artifacts.hasGreenBlob());
            if (largestPurple != null) telemetry.addLine("PURPLE → " + largestPurple);
            if (largestGreen  != null) telemetry.addLine("GREEN  → " + largestGreen);
            telemetry.addLine("[X]=disable  [A]=enable");
            telemetry.update();

            sleep(50);
        }

        visionManager.close();
    }
}
