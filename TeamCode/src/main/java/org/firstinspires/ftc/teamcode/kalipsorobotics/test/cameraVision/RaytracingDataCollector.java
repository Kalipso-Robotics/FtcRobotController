package org.firstinspires.ftc.teamcode.kalipsorobotics.test.cameraVision;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.vision.CameraIntrinsics.CAM_HEIGHT;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.vision.CameraIntrinsics.CAM_WIDTH;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionManager;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionRecognition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblob.ArtifactColorBlobDetectionProcessor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblob.BlobUtils;

/**
 * Ground-truth data collector for CameraIntrinsics raytracing accuracy.
 *
 * Workflow: keep the robot stationary at the origin, place an artifact on the
 * floor at a tape-measured distance, dial that distance in on the gamepad,
 * then record. Repeat at several distances/lateral offsets to build a CSV of
 * (known distance vs. raytraced distance) for error analysis.
 *
 * Controls:
 * - DPad Up/Down:    Known distance +/- 10mm
 * - DPad Left/Right: Known distance +/- 100mm
 * - Left Trigger:    Record current largest-blob reading against known distance
 * - Y Button:        Save last recorded sample to CSV
 */
@TeleOp(name = "Raytracing Data Collector", group = "Test")
public class RaytracingDataCollector extends LinearOpMode {

    private static class GroundTruthSample {
        final double knownDistanceMM;
        final double raytracedDistanceMM;
        final double pixelX, pixelY;
        final double worldX, worldY;
        final String blobLabel;

        GroundTruthSample(double knownDistanceMM, double raytracedDistanceMM,
                          double pixelX, double pixelY, double worldX, double worldY, String blobLabel) {
            this.knownDistanceMM = knownDistanceMM;
            this.raytracedDistanceMM = raytracedDistanceMM;
            this.pixelX = pixelX;
            this.pixelY = pixelY;
            this.worldX = worldX;
            this.worldY = worldY;
            this.blobLabel = blobLabel;
        }

        @Override
        public String toString() {
            return String.format(Locale.US,
                    "known=%.1fmm raytraced=%.1fmm error=%.1fmm px=(%.1f,%.1f) world=(%.1f,%.1f) label=%s",
                    knownDistanceMM, raytracedDistanceMM, raytracedDistanceMM - knownDistanceMM,
                    pixelX, pixelY, worldX, worldY, blobLabel);
        }

        String toCSV() {
            return String.format(Locale.US, "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%s",
                    knownDistanceMM, raytracedDistanceMM, raytracedDistanceMM - knownDistanceMM,
                    pixelX, pixelY, worldX, worldY, blobLabel);
        }
    }

    private static final Position ROBOT_ORIGIN = new Position(0, 0, 0);

    private KFileWriter fileWriter;
    private double knownDistanceMM = 500.0;
    private GroundTruthSample lastSample = null;
    private boolean wasLeftTriggerPressed = false;
    private boolean wasYPressed = false;
    private int savedCount = 0;

    @Override
    public void runOpMode() {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        fileWriter = new KFileWriter("RaytracingGroundTruth", opModeUtilities);
        fileWriter.writeLine("KnownDistanceMM,RaytracedDistanceMM,ErrorMM,PixelX,PixelY,WorldX,WorldY,BlobLabel");

        CameraIntrinsics intrinsics = CameraIntrinsics.ARDUCAM;
        ArtifactColorBlobDetectionProcessor artifacts = new ArtifactColorBlobDetectionProcessor();
        VisionManager visionManager = new VisionManager.Builder(hardwareMap)
                .withResolution(CAM_WIDTH, CAM_HEIGHT)
                .addProcessor(artifacts)
                .streamImmediately()
                .build();

        telemetry.addLine("=== Raytracing Data Collector ===");
        telemetry.addLine("Place artifact at a known distance, dial it in, and record.");
        telemetry.addData("Writing to", fileWriter.getPath());
        telemetry.addLine("Press PLAY to start");
        telemetry.update();

        KLog.d("RaytracingDataCollector", "Writing to " + fileWriter.getPath());
        KLog.d("RaytracingDataCollector", "Initialization complete. Waiting for start...");
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) knownDistanceMM += 10;
            if (gamepad1.dpad_down) knownDistanceMM -= 10;
            if (gamepad1.dpad_right) knownDistanceMM += 100;
            if (gamepad1.dpad_left) knownDistanceMM -= 100;
            knownDistanceMM = Math.max(0, knownDistanceMM);

            List<VisionRecognition> recognitions = artifacts.getLatestResult();
            VisionRecognition largest = BlobUtils.findLargestByArea(recognitions);

            double raytracedDistanceMM = Double.NaN;
            Point pixel = null;
            Point worldPos = null;
            if (largest != null) {
                pixel = largest.getBottomMiddlePixel();
                worldPos = intrinsics.calculateRobotFramePos(pixel.getX(), pixel.getY());
                raytracedDistanceMM = intrinsics.getDistanceFromRobot(pixel.getX(), pixel.getY(), ROBOT_ORIGIN);
            }

            if (gamepad1.left_trigger > 0.1) {
                if (!wasLeftTriggerPressed && largest != null && worldPos != null) {
                    lastSample = new GroundTruthSample(knownDistanceMM, raytracedDistanceMM,
                            pixel.getX(), pixel.getY(), worldPos.getX(), worldPos.getY(), largest.label);
                    KLog.d("RaytracingDataCollector", () -> "RECORDED: " + lastSample);
                    wasLeftTriggerPressed = true;
                }
            } else {
                wasLeftTriggerPressed = false;
            }

            if (gamepad1.y) {
                if (!wasYPressed) {
                    if (lastSample != null) {
                        fileWriter.writeLine(lastSample.toCSV());
                        try {
                            fileWriter.flush();
                        } catch (IOException e) {
                            KLog.e("RaytracingDataCollector", "Failed to flush sample to disk", e);
                        }
                        savedCount++;
                        KLog.d("RaytracingDataCollector", () -> "SAVED TO FILE: " + lastSample);
                    } else {
                        KLog.d("RaytracingDataCollector", "No sample to save!");
                    }
                    wasYPressed = true;
                }
                telemetry.addLine(lastSample != null ? "*** SAVED TO FILE ***" : "*** NO DATA TO SAVE ***");
            } else {
                wasYPressed = false;
            }

            telemetry.addLine("=== Raytracing Data Collector ===");
            telemetry.addData("Known Distance (dial)", "%.1f mm", knownDistanceMM);
            telemetry.addData("Rows Saved", savedCount);
            telemetry.addData("Objects Detected", recognitions == null ? 0 : recognitions.size());
            if (largest != null) {
                telemetry.addData("Largest Blob", largest.formattedLabel);
                telemetry.addData("Bottom-Middle Pixel", "(%.1f, %.1f)", pixel.getX(), pixel.getY());
                if (worldPos != null) {
                    telemetry.addData("Raytraced World Pos", "(%.1f, %.1f)", worldPos.getX(), worldPos.getY());
                    telemetry.addData("Raytraced Distance", "%.1f mm", raytracedDistanceMM);
                } else {
                    telemetry.addLine("Blob above horizon - no raytrace.");
                }
            } else {
                telemetry.addLine("No blob detected.");
            }
            telemetry.addLine();
            if (lastSample != null) {
                telemetry.addLine("--- Last Recorded Sample ---");
                telemetry.addLine(lastSample.toString());
            } else {
                telemetry.addLine("--- No Sample Recorded Yet ---");
            }
            telemetry.addLine();
            telemetry.addLine("DPad Up/Down: known dist +/-10mm");
            telemetry.addLine("DPad Left/Right: known dist +/-100mm");
            telemetry.addLine("Left Trigger: record");
            telemetry.addLine("Y: save to file");
            telemetry.update();
        }

        String path = fileWriter.getPath();
        fileWriter.close();
        visionManager.close();

        KLog.d("RaytracingDataCollector", "Data collection complete. " + savedCount + " rows written to " + path);
        KLog.d("RaytracingDataCollector", "Pull it using: adb pull " + path + " ~/");
    }
}
