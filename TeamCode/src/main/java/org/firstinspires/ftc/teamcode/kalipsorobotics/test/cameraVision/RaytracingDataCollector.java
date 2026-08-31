package org.firstinspires.ftc.teamcode.kalipsorobotics.test.cameraVision;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.vision.CameraIntrinsics.CAM_HEIGHT;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.vision.CameraIntrinsics.CAM_WIDTH;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Vector3d;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionManager;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionRecognition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblobbing.ArtifactColorBlobDetectionProcessor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblobbing.BlobUtils;

/**
 * Ground-truth data collector for CameraIntrinsics raytracing accuracy.
 *
 * WHAT THIS IS FOR:
 *   Two jobs in one CSV.
 *
 *   1. SOLVE THE INTRINSICS. The shipped fx/fy/cx/cy came from a 1280x800
 *      checkerboard calibration that was then rescaled by 0.5 on x but 0.6 on y.
 *      That is only valid if the camera anamorphically squashes the full sensor
 *      into 640x480; if it crops instead, fy is ~19% wrong and every projected
 *      distance is biased. With the camera level at height h, the floor-contact
 *      pixel row obeys
 *
 *          pixelY = cy + fy * (h / d)        -- LINEAR in (1/d)
 *
 *      so regressing pixelY against 1/d recovers cy (intercept) and fy*h (slope)
 *      with nothing but a tape measure. Likewise bboxW = D*fx/d recovers fx, and
 *      pixelX against lateral/d recovers cx. See fit_intrinsics.py.
 *
 *   2. COMPARE THE TWO RANGING METHODS HEAD TO HEAD. Every row carries both the
 *      floor-plane projection AND the known-object-size projection, so the
 *      distance band where each one wins is visible directly in the data.
 *
 * WORKFLOW:
 *   Keep the robot stationary. Place an artifact on the floor at a tape-measured
 *   distance (from ROBOT CENTER, not from the lens -- the camera offsets are
 *   written into every row so the fitting script can convert). Dial the distance
 *   and lateral offset in on the gamepad, pull the trigger to capture a burst,
 *   then press Y to write it. Repeat across ~10 distances.
 *
 *   For the fit set, keep the ball CENTERED (lateral 0) and vary distance from
 *   ~400 mm to ~2500 mm, denser below 1 m where the geometry is steepest. Then
 *   add ~6 off-center samples at 2-3 distances so cx can be solved too.
 *
 *   For the hold-out set, use distances that were NOT in the fit set.
 *
 * WHY BURSTS:
 *   The old version recorded a single frame per trigger, so one lucky frame could
 *   look perfect. Each trigger now captures BURST_FRAMES distinct camera frames,
 *   which makes per-sample noise visible in the CSV instead of hiding it.
 *
 * Controls:
 *   DPad Up/Down:      known distance +/- 10 mm
 *   DPad Right/Left:   known distance +/- 100 mm
 *   Right/Left bumper: known lateral  +/- 10 mm   (+ = LEFT of robot center)
 *   B / X:             known lateral  +/- 100 mm
 *   Left Trigger:      capture a burst of BURST_FRAMES frames
 *   Y:                 write the captured burst to CSV
 *   A:                 discard the captured burst
 */
@Config
@TeleOp(name = "Raytracing Data Collector", group = "Test")
public class RaytracingDataCollector extends LinearOpMode {

    // ── Dashboard tunables ───────────────────────────────────────────────────
    /** Frames captured per trigger pull. */
    public static int  BURST_FRAMES = 30;
    public static long EXPOSURE_MS  = 20;
    public static int  GAIN         = 250;

    /**
     * Mount geometry, live-overridable so a hold-out run can verify freshly
     * fitted constants without a redeploy. Defaults match CameraIntrinsics.ARDUCAM.
     * Note X is NEGATIVE: the ray uses normX = cx - pixelX so +X is LEFT, and the
     * Arducam sits to the RIGHT of robot center.
     */
    public static double MOUNT_ANGLE_DEG = 0.0;
    public static double CAM_HEIGHT_MM   = 236.163;
    public static double CAM_OFFSET_X_MM = -157.548;
    public static double CAM_OFFSET_Z_MM = 151.868;

    private static final String TAG = "RaytracingDataCollector";

    private static final String CSV_HEADER =
            "KnownDistMM,KnownLateralMM,"
            + "PixelBottomX,PixelBottomY,PixelCenterX,PixelCenterY,"
            + "BboxW,BboxH,Aspect,ContourArea,Circularity,Label,BlobCount,"
            + "FloorDistMM,FloorLateralMM,FloorForwardMM,FloorErrMM,"
            + "SizeDistMM,SizeLateralMM,SizeForwardMM,SizeErrMM,"
            + "ObjDiameterMM,MountAngleDeg,CamHeightMM,CamOffsetXMM,CamOffsetZMM,"
            + "Fx,Fy,Cx,Cy,ProcFrame";

    /** One captured camera frame. Held in memory until Y writes the burst. */
    private static class GroundTruthSample {
        final String csv;
        final String summary;
        GroundTruthSample(String csv, String summary) {
            this.csv = csv;
            this.summary = summary;
        }
    }

    private static final Position ROBOT_ORIGIN = new Position(0, 0, 0);

    private double knownDistanceMM = 500.0;
    private double knownLateralMM  = 0.0;

    private final List<GroundTruthSample> burst = new ArrayList<>();
    private boolean capturing = false;
    private boolean wasLeftTriggerPressed = false;
    private int lastCapturedFrame = -1;
    private int savedRows = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        KFileWriter fileWriter = new KFileWriter("RaytracingGroundTruth", opModeUtilities);
        fileWriter.writeLine(CSV_HEADER);

        ArtifactColorBlobDetectionProcessor artifacts = new ArtifactColorBlobDetectionProcessor();
        VisionManager visionManager = new VisionManager.Builder(hardwareMap)
                .withResolution(CAM_WIDTH, CAM_HEIGHT)
                .addProcessor(artifacts)
                .streamImmediately()
                .build();

        // Without a locked exposure the camera auto-adjusts and the HSV thresholds
        // tuned in ArtifactDetectionTest do not transfer. This was missing before.
        visionManager.lockCameraControls(EXPOSURE_MS, GAIN);
        FtcDashboard.getInstance().startCameraStream(visionManager.getPortal(), 30);

        long prevExposure = EXPOSURE_MS;
        int  prevGain     = GAIN;

        telemetry.addLine("=== Raytracing Data Collector ===");
        telemetry.addLine("Place artifact at a known distance, dial it in, and record.");
        telemetry.addData("Writing to", fileWriter.getPath());
        telemetry.addLine("Press PLAY to start");
        telemetry.update();

        KLog.d(TAG, "Writing to " + fileWriter.getPath());
        waitForStart();

        while (opModeIsActive()) {
            if (EXPOSURE_MS != prevExposure || GAIN != prevGain) {
                visionManager.lockCameraControls(EXPOSURE_MS, GAIN);
                prevExposure = EXPOSURE_MS;
                prevGain     = GAIN;
            }

            if (gamepad1.dpadUpWasPressed())    knownDistanceMM += 10;
            if (gamepad1.dpadDownWasPressed())  knownDistanceMM -= 10;
            if (gamepad1.dpadRightWasPressed()) knownDistanceMM += 100;
            if (gamepad1.dpadLeftWasPressed())  knownDistanceMM -= 100;
            knownDistanceMM = Math.max(0, knownDistanceMM);

            if (gamepad1.rightBumperWasPressed()) knownLateralMM += 10;
            if (gamepad1.leftBumperWasPressed())  knownLateralMM -= 10;
            if (gamepad1.bWasPressed())           knownLateralMM += 100;
            if (gamepad1.xWasPressed())           knownLateralMM -= 100;

            // Rebuild each loop so dashboard mount tweaks take effect live.
            CameraIntrinsics intrinsics = CameraIntrinsics.ARDUCAM.withMount(
                    Math.toRadians(MOUNT_ANGLE_DEG),
                    new Vector3d(CAM_OFFSET_X_MM, CAM_HEIGHT_MM, CAM_OFFSET_Z_MM));

            List<VisionRecognition> recognitions = artifacts.getLatestResult();
            VisionRecognition largest = BlobUtils.findLargestByArea(recognitions);
            int blobCount = (recognitions == null) ? 0 : recognitions.size();
            int procFrame = artifacts.getDiagFrameCount();

            // ── Floor-plane projection ───────────────────────────────────────
            Point pixelBottom = null;
            Point floorPos    = null;
            double floorDist  = Double.NaN;
            // ── Known-object-size projection ─────────────────────────────────
            Point sizePos     = null;
            double sizeDist   = Double.NaN;
            double objDiameterMM = artifacts.getObjectDiameterMM();

            if (largest != null) {
                pixelBottom = largest.getBottomMiddlePixel();
                floorPos = intrinsics.calculateRobotFramePos(pixelBottom.getX(), pixelBottom.getY());
                floorDist = intrinsics.getDistanceFromRobot(
                        pixelBottom.getX(), pixelBottom.getY(), ROBOT_ORIGIN);

                sizePos  = intrinsics.calculateRobotFramePosFromSize(largest, objDiameterMM);
                sizeDist = intrinsics.getDistanceFromRobotBySize(largest, objDiameterMM, ROBOT_ORIGIN);
            }

            // ── Burst capture ────────────────────────────────────────────────
            boolean triggerDown = gamepad1.left_trigger > 0.1;
            if (triggerDown && !wasLeftTriggerPressed) {
                burst.clear();
                capturing = true;
                lastCapturedFrame = -1;
                KLog.d(TAG, "Burst started: " + BURST_FRAMES + " frames @ "
                        + knownDistanceMM + "mm / lat " + knownLateralMM + "mm");
            }
            wasLeftTriggerPressed = triggerDown;

            if (capturing && largest != null && procFrame != lastCapturedFrame) {
                lastCapturedFrame = procFrame;
                burst.add(buildSample(largest, pixelBottom, floorPos, floorDist,
                        sizePos, sizeDist, objDiameterMM, blobCount, procFrame, intrinsics));
                if (burst.size() >= BURST_FRAMES) {
                    capturing = false;
                    KLog.d(TAG, "Burst complete: " + burst.size() + " frames captured.");
                }
            }

            if (gamepad1.aWasPressed()) {
                burst.clear();
                capturing = false;
                KLog.d(TAG, "Burst discarded.");
            }

            if (gamepad1.yWasPressed()) {
                if (burst.isEmpty()) {
                    KLog.d(TAG, "No burst to save!");
                } else {
                    for (GroundTruthSample sample : burst) {
                        fileWriter.writeLine(sample.csv);
                        savedRows++;
                    }
                    try {
                        fileWriter.flush();
                    } catch (IOException e) {
                        KLog.e(TAG, "Failed to flush burst to disk", e);
                    }
                    KLog.d(TAG, "SAVED " + burst.size() + " rows (total " + savedRows + ")");
                    burst.clear();
                }
            }

            // ── Telemetry ────────────────────────────────────────────────────
            telemetry.addLine("=== Raytracing Data Collector ===");
            telemetry.addData("Known Distance (dial)", "%.1f mm", knownDistanceMM);
            telemetry.addData("Known Lateral  (dial)", "%.1f mm  (+ = LEFT)", knownLateralMM);
            telemetry.addData("Burst", capturing
                    ? String.format(Locale.US, "CAPTURING %d/%d", burst.size(), BURST_FRAMES)
                    : String.format(Locale.US, "%d held", burst.size()));
            telemetry.addData("Rows Saved", savedRows);
            telemetry.addData("Objects Detected", blobCount);
            telemetry.addLine("-------------------");

            if (largest != null) {
                double aspect = (largest.getHeight() > 0)
                        ? largest.getWidth() / largest.getHeight() : 0.0;
                telemetry.addData("Largest Blob", largest.formattedLabel);
                telemetry.addData("Bottom-Middle Pixel", "(%.1f, %.1f)",
                        pixelBottom.getX(), pixelBottom.getY());
                telemetry.addData("Bbox WxH", "%.0f x %.0f   aspect %.3f  (expect %.3f)",
                        largest.getWidth(), largest.getHeight(), aspect,
                        intrinsics.getExpectedSphereAspect());

                if (floorPos != null) {
                    telemetry.addData("FLOOR  robotFrame", "(lat %.1f, fwd %.1f)",
                            floorPos.getX(), floorPos.getY());
                    telemetry.addData("FLOOR  distance", "%.1f mm   err %+.1f mm",
                            floorDist, floorDist - knownDistanceMM);
                } else {
                    telemetry.addLine("FLOOR  blob above horizon - no projection.");
                }

                if (sizePos != null) {
                    telemetry.addData("SIZE   robotFrame", "(lat %.1f, fwd %.1f)",
                            sizePos.getX(), sizePos.getY());
                    telemetry.addData("SIZE   distance", "%.1f mm   err %+.1f mm",
                            sizeDist, sizeDist - knownDistanceMM);
                } else {
                    telemetry.addLine("SIZE   no bbox - no projection.");
                }
            } else {
                telemetry.addLine("No blob detected.");
            }

            telemetry.addLine("-------------------");
            if (!burst.isEmpty()) {
                telemetry.addLine("Last captured: " + burst.get(burst.size() - 1).summary);
            }
            telemetry.addLine("DPad U/D: dist +/-10   DPad L/R: dist +/-100");
            telemetry.addLine("Bumpers: lateral +/-10   [B]/[X]: lateral +/-100");
            telemetry.addLine("LT: capture burst   [Y]: save burst   [A]: discard");
            telemetry.update();
        }

        String path = fileWriter.getPath();
        fileWriter.close();
        FtcDashboard.getInstance().stopCameraStream();
        visionManager.close();

        KLog.d(TAG, "Data collection complete. " + savedRows + " rows written to " + path);
        KLog.d(TAG, "Pull it using: adb pull " + path + " ~/");
    }

    private GroundTruthSample buildSample(VisionRecognition blob, Point pixelBottom,
                                          Point floorPos, double floorDist,
                                          Point sizePos, double sizeDist,
                                          double objDiameterMM, int blobCount, int procFrame,
                                          CameraIntrinsics intrinsics) {
        double aspect = (blob.getHeight() > 0) ? blob.getWidth() / blob.getHeight() : 0.0;

        double floorLat = (floorPos == null) ? Double.NaN : floorPos.getX();
        double floorFwd = (floorPos == null) ? Double.NaN : floorPos.getY();
        double floorErr = Double.isNaN(floorDist) ? Double.NaN : floorDist - knownDistanceMM;

        double sizeLat = (sizePos == null) ? Double.NaN : sizePos.getX();
        double sizeFwd = (sizePos == null) ? Double.NaN : sizePos.getY();
        double sizeErr = Double.isNaN(sizeDist) ? Double.NaN : sizeDist - knownDistanceMM;

        String csv = String.format(Locale.US,
                "%.1f,%.1f,"
                + "%.2f,%.2f,%.2f,%.2f,"
                + "%.1f,%.1f,%.4f,%.1f,%.4f,%s,%d,"
                + "%.2f,%.2f,%.2f,%.2f,"
                + "%.2f,%.2f,%.2f,%.2f,"
                + "%.2f,%.4f,%.3f,%.3f,%.3f,"
                + "%.5f,%.5f,%.5f,%.5f,%d",
                knownDistanceMM, knownLateralMM,
                pixelBottom.getX(), pixelBottom.getY(),
                blob.center.getX(), blob.center.getY(),
                blob.getWidth(), blob.getHeight(), aspect,
                blob.getArea(), blob.getCircularity(), blob.label, blobCount,
                floorDist, floorLat, floorFwd, floorErr,
                sizeDist, sizeLat, sizeFwd, sizeErr,
                objDiameterMM, MOUNT_ANGLE_DEG, CAM_HEIGHT_MM, CAM_OFFSET_X_MM, CAM_OFFSET_Z_MM,
                intrinsics.getFx(), intrinsics.getFy(), intrinsics.getCx(), intrinsics.getCy(),
                procFrame);

        String summary = String.format(Locale.US,
                "known=%.0f floor=%.0f (%+.0f) size=%.0f (%+.0f) px=(%.0f,%.0f) %s",
                knownDistanceMM, floorDist, floorErr, sizeDist, sizeErr,
                pixelBottom.getX(), pixelBottom.getY(), blob.label);

        return new GroundTruthSample(csv, summary);
    }
}
