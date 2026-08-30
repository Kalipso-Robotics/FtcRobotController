package org.firstinspires.ftc.teamcode.kalipsorobotics.test.cameraVision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionManager;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionRecognition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblobbing.ArtifactColorBlobDetectionProcessor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblobbing.KColorBlobProcessor;

import java.io.IOException;
import java.util.List;
import java.util.Locale;

/**
 * Live tuning OpMode + false-positive measuring rig for the artifact color blob pipeline.
 *
 * WHY THIS EXISTS:
 *   Pointing the camera at ONE ball was producing several detections, with background
 *   clutter passing as balls. "Looks better on the feed" is not a result, so this OpMode
 *   turns it into a number: the fraction of camera frames on which exactly one blob of
 *   the expected color was reported.
 *
 * TUNE WITHOUT REDEPLOYING:
 *   1. Connect to the robot WiFi, open http://192.168.43.1:8080 (FTC Dashboard).
 *   2. Expand "ArtifactDetectionTest" in the variable panel.
 *   3. Every gate below is live - HSV bounds, area, circularity, aspect, exposure.
 *   4. The camera stream is mirrored into the dashboard, so you can see the effect.
 *   5. When it looks right, paste the winners into ArtifactColorBlobDetectionProcessor
 *      (HSV) and KColorBlobProcessor (gates).
 *
 * MEASURE THE FALSE-POSITIVE RATE:
 *   Set CONDITION (e.g. "purple_1.0m_center") and EXPECT_LABEL, place exactly one ball,
 *   then hold [A] to record ~100 camera frames. Rows are appended to a CSV. Repeat for
 *   each of the 12 conditions: {purple,green} x {0.5m,1.0m,1.5m} x {center,+30deg,-30deg}.
 *
 *   PASS = single-correct-blob rate >= 98% on every condition.
 *
 *   Rows are deduplicated on the processor's own frame counter, so one row really is
 *   one camera frame, not one OpMode loop.
 *
 * READ THE ASPECT COLUMN - it is a free intrinsics check:
 *   A sphere projects to fx*a wide by fy*a tall, so bbox w/h should read fx/fy = 0.835.
 *   If real balls read ~1.00 instead, the 1280x800 -> 640x480 rescale in CameraIntrinsics
 *   is wrong and fy is off by ~19%. Carry that straight into fit_intrinsics.py.
 *
 * Gamepad:
 *   [A] (hold)  record frames for the current CONDITION
 *   [B]         start a new condition block (resets the pass-rate counters)
 *   [X]         disable processor (see the raw camera feed)
 *   [Y]         re-enable processor
 *   [DPad Up]   toggle annotated JPEG snapshots (rejected contours drawn in red)
 */
@Config
@TeleOp(name = "Test: Artifact Detection", group = "Test Vision")
public class ArtifactDetectionTest extends LinearOpMode {

    // -- Camera --------------------------------------------------------------
    public static long EXPOSURE_MS = 20;
    public static int  GAIN        = 250;

    // -- Blob gates (mirror of KColorBlobProcessor defaults) -----------------
    // These START at the shipped values so the OpMode measures the CURRENT
    // behaviour first. Suggested values to try live are noted per line.
    public static double MIN_AREA         = 250;
    public static double MAX_AREA         = 30_000;
    public static double MIN_CIRCULARITY  = 0.55;   // try 0.75 (spheres measure 0.85-0.90)
    public static double EXPECTED_ASPECT  = 0.834;  // = fx/fy
    public static double ASPECT_TOLERANCE = 0;      // 0 = gate OFF; try 0.30 to enable

    // -- HSV bounds (OpenCV scale: H 0-180, S 0-255, V 0-255) ----------------
    // Purple S_min = 16 is the prime suspect for the background false positives:
    // at that saturation floor, near-grey pixels pass as purple. Try S_min 80,
    // V_min 50 -- raise them here on the dashboard and watch the pass rate.
    public static double PURPLE_H_MIN = 135, PURPLE_S_MIN =  16, PURPLE_V_MIN =  40;
    public static double PURPLE_H_MAX = 180, PURPLE_S_MAX = 255, PURPLE_V_MAX = 255;
    public static double GREEN_H_MIN  =  69, GREEN_S_MIN  = 108, GREEN_V_MIN  =  38;
    public static double GREEN_H_MAX  =  90, GREEN_S_MAX  = 255, GREEN_V_MAX  = 255;

    // -- Test bookkeeping ----------------------------------------------------
    /** Free-text tag written into every CSV row. e.g. "purple_1.0m_center". */
    public static String CONDITION    = "purple_1.0m_center";
    /** The colour that SHOULD be seen. A frame passes only if it sees exactly one of these. */
    public static String EXPECT_LABEL = "Purple";
    /** Snapshot cadence used when snapshots are toggled on with DPad Up. */
    public static int    SNAPSHOT_EVERY_N = 5;

    private static final String TAG = "ArtifactDetectionTest";

    private int framesRecorded = 0;
    private int framesPassing  = 0;
    private int lastLoggedFrame = -1;
    private boolean snapshotsOn = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        KFileWriter fileWriter = new KFileWriter("BlobFalsePositive", opModeUtilities);
        fileWriter.writeLine("Condition,ExpectLabel,ProcFrame,BlobCount,Pass,"
                + "Label,ContourArea,Circularity,W,H,Aspect,CenterX,CenterY,"
                + "RawContours,AreaRej,CircRej,AspectRej,"
                + "MinArea,MaxArea,MinCirc,ExpAspect,AspectTol,"
                + "PurpleHMin,PurpleSMin,PurpleVMin,GreenHMin,GreenSMin,GreenVMin,ExposureMs,Gain");

        ArtifactColorBlobDetectionProcessor artifacts = new ArtifactColorBlobDetectionProcessor();

        VisionManager visionManager = new VisionManager.Builder(hardwareMap)
                .addProcessor(artifacts)
                .streamImmediately()
                .build();

        visionManager.lockCameraControls(EXPOSURE_MS, GAIN);
        FtcDashboard.getInstance().startCameraStream(visionManager.getPortal(), 30);

        long prevExposure = EXPOSURE_MS;
        int  prevGain     = GAIN;

        telemetry.addLine("=== Artifact Detection Test ===");
        telemetry.addData("CSV", fileWriter.getPath());
        telemetry.addLine("Dashboard: http://192.168.43.1:8080");
        telemetry.addLine("Press PLAY, then hold [A] to record a condition.");
        telemetry.update();
        KLog.d(TAG, "Writing to " + fileWriter.getPath());

        waitForStart();

        while (opModeIsActive()) {

            // Re-apply camera controls only when the dashboard values actually change -
            // lockCameraControls busy-waits for STREAMING, so calling it every loop stalls.
            if (EXPOSURE_MS != prevExposure || GAIN != prevGain) {
                visionManager.lockCameraControls(EXPOSURE_MS, GAIN);
                prevExposure = EXPOSURE_MS;
                prevGain     = GAIN;
            }

            pushGates(artifacts);

            if (gamepad1.x) visionManager.disable(artifacts);
            if (gamepad1.y) visionManager.enable(artifacts);

            if (gamepad1.dpadUpWasPressed()) {
                snapshotsOn = !snapshotsOn;
                artifacts.enableSnapshotSaving(snapshotsOn ? SNAPSHOT_EVERY_N : 0);
            }

            if (gamepad1.bWasPressed()) {
                framesRecorded = 0;
                framesPassing  = 0;
                lastLoggedFrame = -1;
                KLog.d(TAG, "New condition block: " + CONDITION);
            }

            List<VisionRecognition> blobs = artifacts.getLatestResult();
            int blobCount = (blobs == null) ? 0 : blobs.size();

            // One CSV row per CAMERA frame, not per OpMode loop. The processor's own
            // counter is the only honest frame clock available here.
            int procFrame = artifacts.getDiagFrameCount();
            boolean newFrame = (procFrame != lastLoggedFrame);

            boolean pass = (blobCount == 1)
                    && blobs != null
                    && EXPECT_LABEL.equals(blobs.get(0).label);

            if (gamepad1.a && newFrame) {
                lastLoggedFrame = procFrame;
                framesRecorded++;
                if (pass) framesPassing++;
                writeRows(fileWriter, artifacts, blobs, blobCount, procFrame, pass);
            }

            double passRate = (framesRecorded == 0)
                    ? 0.0 : (100.0 * framesPassing / framesRecorded);

            telemetry.addLine("=== Artifact Detection Test ===");
            telemetry.addData("CONDITION", CONDITION);
            telemetry.addData("Expect", EXPECT_LABEL);
            telemetry.addData("Recorded", "%d frames", framesRecorded);
            telemetry.addData("Single-correct-blob rate", "%.1f%%  (target >= 98%%)", passRate);
            telemetry.addData("Snapshots", snapshotsOn ? "ON every " + SNAPSHOT_EVERY_N : "off");
            telemetry.addLine("-------------------");
            telemetry.addData("Blobs this frame", blobCount);
            if (blobs != null) {
                for (VisionRecognition r : blobs) {
                    double aspect = (r.getHeight() > 0) ? r.getWidth() / r.getHeight() : 0.0;
                    telemetry.addLine(String.format(Locale.US,
                            "  %s a=%.0f c=%.2f %.0fx%.0f ar=%.2f px=(%.0f,%.0f)",
                            r.label, r.getArea(), r.getCircularity(),
                            r.getWidth(), r.getHeight(), aspect,
                            r.center.getX(), r.center.getY()));
                }
            }
            telemetry.addLine("-------------------");
            telemetry.addData("Pipeline", artifacts.getDiagnosticSummary());
            telemetry.addLine("[A]=record  [B]=new condition  [X]/[Y]=disable/enable");
            telemetry.addLine("[DPad Up]=toggle snapshots");
            telemetry.update();
        }

        try {
            fileWriter.flush();
        } catch (IOException e) {
            KLog.e(TAG, "Failed to flush CSV", e);
        }
        String path = fileWriter.getPath();
        fileWriter.close();
        visionManager.close();
        KLog.d(TAG, "Done. " + framesRecorded + " frames written to " + path);
        KLog.d(TAG, "Pull it using: adb pull " + path + " ~/");
    }

    /** Push every dashboard gate into the live processor. */
    private void pushGates(ArtifactColorBlobDetectionProcessor artifacts) {
        artifacts.setMinContourArea(MIN_AREA);
        artifacts.setMaxContourArea(MAX_AREA);
        artifacts.setMinCircularity(MIN_CIRCULARITY);
        artifacts.setExpectedAspect(EXPECTED_ASPECT);
        artifacts.setAspectTolerance(ASPECT_TOLERANCE);

        // Scalar.val[] is mutable, so HSV bounds can be retuned in place without
        // rebuilding the channels. Null until the portal has called onInit().
        KColorBlobProcessor.ColorChannel[] channels = artifacts.getChannels();
        if (channels == null || channels.length < 2) return;
        setHsv(channels[0], PURPLE_H_MIN, PURPLE_S_MIN, PURPLE_V_MIN,
                            PURPLE_H_MAX, PURPLE_S_MAX, PURPLE_V_MAX);
        setHsv(channels[1], GREEN_H_MIN,  GREEN_S_MIN,  GREEN_V_MIN,
                            GREEN_H_MAX,  GREEN_S_MAX,  GREEN_V_MAX);
    }

    private static void setHsv(KColorBlobProcessor.ColorChannel channel,
                               double hMin, double sMin, double vMin,
                               double hMax, double sMax, double vMax) {
        channel.hsvLowerBound.val[0] = hMin;
        channel.hsvLowerBound.val[1] = sMin;
        channel.hsvLowerBound.val[2] = vMin;
        channel.hsvUpperBound.val[0] = hMax;
        channel.hsvUpperBound.val[1] = sMax;
        channel.hsvUpperBound.val[2] = vMax;
    }

    /**
     * One row per blob, so extra detections are visible individually rather than
     * collapsed into a count. A frame with zero blobs still writes one row so the
     * denominator stays honest.
     */
    private void writeRows(KFileWriter fileWriter,
                           ArtifactColorBlobDetectionProcessor artifacts,
                           List<VisionRecognition> blobs,
                           int blobCount, int procFrame, boolean pass) {
        String tail = String.format(Locale.US,
                ",%d,%d,%d,%d,%.0f,%.0f,%.2f,%.3f,%.2f,"
                        + "%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%d,%d",
                artifacts.getDiagRawContours(), artifacts.getDiagAreaRejected(),
                artifacts.getDiagCircRejected(), artifacts.getDiagAspectRejected(),
                MIN_AREA, MAX_AREA, MIN_CIRCULARITY, EXPECTED_ASPECT, ASPECT_TOLERANCE,
                PURPLE_H_MIN, PURPLE_S_MIN, PURPLE_V_MIN,
                GREEN_H_MIN, GREEN_S_MIN, GREEN_V_MIN,
                EXPOSURE_MS, GAIN);

        String head = String.format(Locale.US, "%s,%s,%d,%d,%d",
                CONDITION, EXPECT_LABEL, procFrame, blobCount, pass ? 1 : 0);

        if (blobs == null || blobs.isEmpty()) {
            fileWriter.writeLine(head + ",NONE,0,0,0,0,0,0,0" + tail);
            return;
        }
        for (VisionRecognition r : blobs) {
            double aspect = (r.getHeight() > 0) ? r.getWidth() / r.getHeight() : 0.0;
            fileWriter.writeLine(head + String.format(Locale.US,
                    ",%s,%.1f,%.3f,%.1f,%.1f,%.3f,%.1f,%.1f",
                    r.label, r.getArea(), r.getCircularity(),
                    r.getWidth(), r.getHeight(), aspect,
                    r.center.getX(), r.center.getY()) + tail);
        }
    }
}
