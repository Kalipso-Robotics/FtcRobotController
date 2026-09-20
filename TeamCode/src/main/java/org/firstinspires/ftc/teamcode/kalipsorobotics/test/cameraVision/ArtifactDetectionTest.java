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
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Guided trial runner + false-positive measuring rig for the artifact color blob pipeline.
 *
 * WHY THIS EXISTS:
 *   Pointing the camera at ONE ball was producing several detections, with background
 *   clutter passing as balls. "Looks better on the feed" is not a result, so this OpMode
 *   turns it into a number: the fraction of camera frames on which exactly one blob of
 *   the expected color was reported.
 *
 * HOW A SESSION RUNS:
 *   The condition matrix lives in TRIALS below and the OpMode walks you through it one
 *   entry at a time. For each trial:
 *
 *     1. Telemetry names the condition and the ball color to place. Place exactly one ball.
 *     2. Press [A]. It records TARGET_FRAMES distinct camera frames and stops on its own
 *        (~3.5s at 30fps) - no holding a button.
 *     3. It shows the pass rate and verdict for that trial.
 *     4. Press [A] to commit and advance, or [B] to throw the trial away and redo it.
 *
 *   PASS = single-correct-blob rate >= PASS_RATE_TARGET on every condition.
 *
 *   Rows are deduplicated on the processor's own frame counter, so one row really is
 *   one camera frame, not one OpMode loop. A trial's rows are held in memory until [A]
 *   commits them, so a redo leaves nothing behind in the CSV.
 *
 * WHAT COMES OUT:
 *   BlobFalsePositive_<ts>.csv   one row per blob per frame (the raw data)
 *   BlobTrialSummary_<ts>.csv    one row per trial (the file you actually read)
 *   logcat, tag KLog_ArtifactDetectionTest   one line per trial, plus a session block
 *
 *   The summary's ZeroBlob / MultiBlob / WrongLabel columns say WHY a condition failed:
 *   nothing detected, background false positives, or color confusion. That is the
 *   difference between raising PURPLE_S_MIN and lowering MIN_AREA.
 *
 * TUNE WITHOUT REDEPLOYING:
 *   1. Connect to the robot WiFi, open http://192.168.43.1:8080 (FTC Dashboard).
 *   2. Expand "ArtifactDetectionTest" in the variable panel.
 *   3. Every gate is live - HSV bounds, area, circularity, aspect, exposure.
 *   4. Gates are FROZEN while a trial records, so a mid-trial tweak can never mix two
 *      configurations into one measurement. Tune between trials, not during.
 *   5. When it looks right, paste the winners into ArtifactColorBlobDetectionProcessor
 *      (HSV) and KColorBlobProcessor (gates).
 *
 * READ THE ASPECT COLUMN - it is a free intrinsics check:
 *   A sphere projects to fx*a wide by fy*a tall, so bbox w/h should read fx/fy = 0.835.
 *   If real balls read ~1.00 instead, the 1280x800 -> 640x480 rescale in CameraIntrinsics
 *   is wrong and fy is off by ~19%. Carry that straight into fit_intrinsics.py.
 *
 * Gamepad:
 *   [A]         start the armed trial / commit a finished trial and advance
 *   [B]         discard the current trial and re-arm it (redo)
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

    // -- Trial control -------------------------------------------------------
    /** Camera frames recorded per trial. 100 gives 1% resolution on the pass rate. */
    public static int    TARGET_FRAMES    = 100;
    /** A trial passes when its single-correct-blob rate reaches this percentage. */
    public static double PASS_RATE_TARGET = 98.0;
    /** Snapshot cadence used when snapshots are toggled on with DPad Up. */
    public static int    SNAPSHOT_EVERY_N = 5;

    /**
     * The condition matrix: {conditionLabel, expectedColorLabel}.
     *
     * {purple, green} x {0.5, 1.0, 1.5 m} x {center, +30deg, -30deg} = 18 trials.
     * (An earlier version of this comment said 12, which is simply wrong: 2*3*3 = 18.)
     *
     * Ordered so you finish every purple placement before switching balls. Delete rows
     * to shorten a session - nothing else depends on the length or the ordering.
     */
    private static final String[][] TRIALS = {
        {"purple_0.5m_center", "Purple"},
        {"purple_0.5m_+30deg", "Purple"},
        {"purple_0.5m_-30deg", "Purple"},
        {"purple_1.0m_center", "Purple"},
        {"purple_1.0m_+30deg", "Purple"},
        {"purple_1.0m_-30deg", "Purple"},
        {"purple_1.5m_center", "Purple"},
        {"purple_1.5m_+30deg", "Purple"},
        {"purple_1.5m_-30deg", "Purple"},
        {"green_0.5m_center",  "Green"},
        {"green_0.5m_+30deg",  "Green"},
        {"green_0.5m_-30deg",  "Green"},
        {"green_1.0m_center",  "Green"},
        {"green_1.0m_+30deg",  "Green"},
        {"green_1.0m_-30deg",  "Green"},
        {"green_1.5m_center",  "Green"},
        {"green_1.5m_+30deg",  "Green"},
        {"green_1.5m_-30deg",  "Green"},
    };

    private static final String TAG = "ArtifactDetectionTest";

    /** Warn on telemetry if a recording trial sees no new camera frame for this long. */
    private static final long STALL_WARN_MS = 2_000;

    private enum State { ARMED, RECORDING, DONE, COMPLETE }

    private static final String FRAME_CSV_HEADER =
            "TrialIndex,Condition,ExpectLabel,ProcFrame,BlobCount,Pass,"
            + "Label,ContourArea,Circularity,W,H,Aspect,CenterX,CenterY,"
            + "RawContours,AreaRej,CircRej,AspectRej,"
            + "MinArea,MaxArea,MinCirc,ExpAspect,AspectTol,"
            + "PurpleHMin,PurpleSMin,PurpleVMin,GreenHMin,GreenSMin,GreenVMin,ExposureMs,Gain";

    private static final String SUMMARY_CSV_HEADER =
            "TrialIndex,Condition,ExpectLabel,Frames,PassFrames,PassRate,Verdict,"
            + "ZeroBlobFrames,MultiBlobFrames,WrongLabelFrames,"
            + "MeanBlobCount,MeanArea,MeanCirc,MeanAspect,MeanRawContours,"
            + "MinArea,MaxArea,MinCirc,ExpAspect,AspectTol,"
            + "PurpleHMin,PurpleSMin,PurpleVMin,GreenHMin,GreenSMin,GreenVMin,ExposureMs,Gain";

    // -- Session state -------------------------------------------------------
    private State state = State.ARMED;
    private int trialIndex = 0;
    private int trialsPassing = 0;
    private final List<String> sessionResults = new ArrayList<>();

    // -- Current-trial state -------------------------------------------------
    private final List<String> rowBuffer = new ArrayList<>();
    private int framesRecorded, framesPassing;
    private int zeroBlobFrames, multiBlobFrames, wrongLabelFrames;
    private double sumBlobCount, sumRawContours;
    private double sumArea, sumCirc, sumAspect;
    private int blobStatSamples;
    private int lastLoggedFrame = -1;
    private long lastFrameMillis;
    private boolean snapshotsOn = false;

    // Gate values frozen at trial start, so the summary row describes the
    // configuration the trial was actually measured under.
    private double fMinArea, fMaxArea, fMinCirc, fExpAspect, fAspectTol;
    private double fPurpleH, fPurpleS, fPurpleV, fGreenH, fGreenS, fGreenV;
    private long   fExposure;
    private int    fGain;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        KFileWriter frameWriter = new KFileWriter("BlobFalsePositive", opModeUtilities);
        frameWriter.writeLine(FRAME_CSV_HEADER);
        KFileWriter summaryWriter = new KFileWriter("BlobTrialSummary", opModeUtilities);
        summaryWriter.writeLine(SUMMARY_CSV_HEADER);

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
        telemetry.addData("Trials", TRIALS.length);
        telemetry.addData("Frame CSV", frameWriter.getPath());
        telemetry.addData("Summary CSV", summaryWriter.getPath());
        telemetry.addLine("Dashboard: http://192.168.43.1:8080");
        telemetry.addLine("Press PLAY, then follow the trial prompts.");
        telemetry.update();
        KLog.d(TAG, "Frame CSV:   " + frameWriter.getPath());
        KLog.d(TAG, "Summary CSV: " + summaryWriter.getPath());

        waitForStart();

        while (opModeIsActive()) {

            // Re-apply camera controls only when the dashboard values actually change -
            // lockCameraControls busy-waits for STREAMING, so calling it every loop stalls.
            // Never mid-trial: that would change the exposure the trial is measuring.
            if (state != State.RECORDING && (EXPOSURE_MS != prevExposure || GAIN != prevGain)) {
                visionManager.lockCameraControls(EXPOSURE_MS, GAIN);
                prevExposure = EXPOSURE_MS;
                prevGain     = GAIN;
            }

            // Gates are live between trials and frozen during one.
            if (state != State.RECORDING) pushGates(artifacts);

            if (gamepad1.x) visionManager.disable(artifacts);
            if (gamepad1.y) visionManager.enable(artifacts);

            if (gamepad1.dpadUpWasPressed()) {
                snapshotsOn = !snapshotsOn;
                artifacts.enableSnapshotSaving(snapshotsOn ? SNAPSHOT_EVERY_N : 0);
            }

            List<VisionRecognition> blobs = artifacts.getLatestResult();
            int blobCount = (blobs == null) ? 0 : blobs.size();
            int procFrame = artifacts.getDiagFrameCount();

            switch (state) {
                case ARMED:
                    if (gamepad1.aWasPressed()) startTrial(artifacts);
                    break;

                case RECORDING:
                    // Checked before the frame is consumed so an abort cannot be
                    // overtaken by the trial completing in the same loop.
                    if (gamepad1.bWasPressed()) {
                        rearmTrial("aborted mid-recording");
                        break;
                    }
                    if (procFrame != lastLoggedFrame) {
                        lastLoggedFrame = procFrame;
                        lastFrameMillis = System.currentTimeMillis();
                        recordFrame(artifacts, blobs, blobCount, procFrame);
                        if (framesRecorded >= Math.max(1, TARGET_FRAMES)) {
                            state = State.DONE;
                            KLog.d(TAG, String.format(Locale.US,
                                    "TRIAL %02d/%02d %s recorded %d frames - %.1f%% (awaiting [A] to commit)",
                                    trialIndex + 1, TRIALS.length, condition(),
                                    framesRecorded, passRate()));
                        }
                    }
                    break;

                case DONE:
                    if (gamepad1.aWasPressed()) {
                        commitTrial(frameWriter, summaryWriter);
                    } else if (gamepad1.bWasPressed()) {
                        rearmTrial("redo requested");
                    }
                    break;

                case COMPLETE:
                default:
                    break;
            }

            renderTelemetry(artifacts, blobs, blobCount);
            telemetry.update();
        }

        try {
            frameWriter.flush();
            summaryWriter.flush();
        } catch (IOException e) {
            KLog.e(TAG, "Failed to flush CSVs", e);
        }
        String framePath   = frameWriter.getPath();
        String summaryPath = summaryWriter.getPath();
        frameWriter.close();
        summaryWriter.close();
        FtcDashboard.getInstance().stopCameraStream();
        visionManager.close();

        logSessionBlock();
        KLog.d(TAG, "Pull them using: adb pull " + framePath + " ~/");
        KLog.d(TAG, "Pull them using: adb pull " + summaryPath + " ~/");
    }

    // -------------------------------------------------------------------------
    // Trial lifecycle
    // -------------------------------------------------------------------------

    private String condition()   { return TRIALS[trialIndex][0]; }
    private String expectLabel() { return TRIALS[trialIndex][1]; }

    private double passRate() {
        return (framesRecorded == 0) ? 0.0 : (100.0 * framesPassing / framesRecorded);
    }

    private boolean trialPassed() { return passRate() >= PASS_RATE_TARGET; }

    private void startTrial(ArtifactColorBlobDetectionProcessor artifacts) {
        resetTrialCounters();
        // Start on the NEXT camera frame, not whatever frame happens to be latched
        // right now - that one was captured before the ball was in position.
        lastLoggedFrame = artifacts.getDiagFrameCount();
        lastFrameMillis = System.currentTimeMillis();
        freezeGates();
        state = State.RECORDING;
        KLog.d(TAG, String.format(Locale.US, "TRIAL %02d/%02d %s expect=%s RECORDING %d frames",
                trialIndex + 1, TRIALS.length, condition(), expectLabel(),
                Math.max(1, TARGET_FRAMES)));
    }

    private void rearmTrial(String why) {
        resetTrialCounters();
        state = State.ARMED;
        KLog.d(TAG, String.format(Locale.US, "TRIAL %02d/%02d %s DISCARDED (%s) - re-armed",
                trialIndex + 1, TRIALS.length, condition(), why));
    }

    private void resetTrialCounters() {
        rowBuffer.clear();
        framesRecorded = 0;
        framesPassing = 0;
        zeroBlobFrames = 0;
        multiBlobFrames = 0;
        wrongLabelFrames = 0;
        sumBlobCount = 0;
        sumRawContours = 0;
        sumArea = 0;
        sumCirc = 0;
        sumAspect = 0;
        blobStatSamples = 0;
        lastLoggedFrame = -1;
    }

    private void freezeGates() {
        fMinArea   = MIN_AREA;
        fMaxArea   = MAX_AREA;
        fMinCirc   = MIN_CIRCULARITY;
        fExpAspect = EXPECTED_ASPECT;
        fAspectTol = ASPECT_TOLERANCE;
        fPurpleH   = PURPLE_H_MIN;
        fPurpleS   = PURPLE_S_MIN;
        fPurpleV   = PURPLE_V_MIN;
        fGreenH    = GREEN_H_MIN;
        fGreenS    = GREEN_S_MIN;
        fGreenV    = GREEN_V_MIN;
        fExposure  = EXPOSURE_MS;
        fGain      = GAIN;
    }

    /**
     * One buffered row per blob, so extra detections are visible individually rather
     * than collapsed into a count. A frame with zero blobs still writes one row so the
     * denominator stays honest. Nothing reaches disk until [A] commits the trial.
     */
    private void recordFrame(ArtifactColorBlobDetectionProcessor artifacts,
                             List<VisionRecognition> blobs, int blobCount, int procFrame) {
        String expect = expectLabel();
        boolean pass = (blobCount == 1) && expect.equals(blobs.get(0).label);

        framesRecorded++;
        if (pass) framesPassing++;
        if (blobCount == 0) {
            zeroBlobFrames++;
        } else if (blobCount > 1) {
            multiBlobFrames++;
        } else if (!pass) {
            wrongLabelFrames++;
        }

        sumBlobCount   += blobCount;
        sumRawContours += artifacts.getDiagRawContours();

        String tail = String.format(Locale.US,
                ",%d,%d,%d,%d,%.0f,%.0f,%.2f,%.3f,%.2f,"
                        + "%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%d,%d",
                artifacts.getDiagRawContours(), artifacts.getDiagAreaRejected(),
                artifacts.getDiagCircRejected(), artifacts.getDiagAspectRejected(),
                fMinArea, fMaxArea, fMinCirc, fExpAspect, fAspectTol,
                fPurpleH, fPurpleS, fPurpleV, fGreenH, fGreenS, fGreenV,
                fExposure, fGain);

        String head = String.format(Locale.US, "%d,%s,%s,%d,%d,%d",
                trialIndex + 1, condition(), expect, procFrame, blobCount, pass ? 1 : 0);

        if (blobs == null || blobs.isEmpty()) {
            rowBuffer.add(head + ",NONE,0,0,0,0,0,0,0" + tail);
            return;
        }
        for (VisionRecognition r : blobs) {
            double aspect = (r.getHeight() > 0) ? r.getWidth() / r.getHeight() : 0.0;
            sumArea   += r.getArea();
            sumCirc   += r.getCircularity();
            sumAspect += aspect;
            blobStatSamples++;
            rowBuffer.add(head + String.format(Locale.US,
                    ",%s,%.1f,%.3f,%.1f,%.1f,%.3f,%.1f,%.1f",
                    r.label, r.getArea(), r.getCircularity(),
                    r.getWidth(), r.getHeight(), aspect,
                    r.center.getX(), r.center.getY()) + tail);
        }
    }

    /** Flush the buffered trial to both CSVs, log it, and advance. */
    private void commitTrial(KFileWriter frameWriter, KFileWriter summaryWriter) {
        for (String row : rowBuffer) {
            frameWriter.writeLine(row);
        }

        double rate = passRate();
        boolean passed = trialPassed();
        double meanBlobCount   = safeMean(sumBlobCount, framesRecorded);
        double meanRawContours = safeMean(sumRawContours, framesRecorded);
        double meanArea        = safeMean(sumArea, blobStatSamples);
        double meanCirc        = safeMean(sumCirc, blobStatSamples);
        double meanAspect      = safeMean(sumAspect, blobStatSamples);

        summaryWriter.writeLine(String.format(Locale.US,
                "%d,%s,%s,%d,%d,%.2f,%s,%d,%d,%d,%.3f,%.1f,%.3f,%.3f,%.2f,"
                        + "%.0f,%.0f,%.2f,%.3f,%.2f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%d,%d",
                trialIndex + 1, condition(), expectLabel(),
                framesRecorded, framesPassing, rate, passed ? "PASS" : "FAIL",
                zeroBlobFrames, multiBlobFrames, wrongLabelFrames,
                meanBlobCount, meanArea, meanCirc, meanAspect, meanRawContours,
                fMinArea, fMaxArea, fMinCirc, fExpAspect, fAspectTol,
                fPurpleH, fPurpleS, fPurpleV, fGreenH, fGreenS, fGreenV,
                fExposure, fGain));

        try {
            frameWriter.flush();
            summaryWriter.flush();
        } catch (IOException e) {
            KLog.e(TAG, "Failed to flush trial " + (trialIndex + 1), e);
        }

        String line = String.format(Locale.US,
                "TRIAL %02d/%02d %-20s %5.1f%% %s  zero=%d multi=%d wrong=%d aspect=%.3f",
                trialIndex + 1, TRIALS.length, condition(), rate, passed ? "PASS" : "FAIL",
                zeroBlobFrames, multiBlobFrames, wrongLabelFrames, meanAspect);
        KLog.d(TAG, line);
        sessionResults.add(line);
        if (passed) trialsPassing++;

        resetTrialCounters();
        trialIndex++;
        if (trialIndex >= TRIALS.length) {
            state = State.COMPLETE;
            logSessionBlock();
        } else {
            state = State.ARMED;
        }
    }

    private static double safeMean(double sum, int n) { return (n == 0) ? 0.0 : sum / n; }

    private void logSessionBlock() {
        if (sessionResults.isEmpty()) return;
        KLog.d(TAG, "===== SESSION SUMMARY =====");
        for (String line : sessionResults) {
            KLog.d(TAG, line);
        }
        KLog.d(TAG, String.format(Locale.US, "%d/%d trials passing (target >= %.1f%%)",
                trialsPassing, sessionResults.size(), PASS_RATE_TARGET));
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    private void renderTelemetry(ArtifactColorBlobDetectionProcessor artifacts,
                                 List<VisionRecognition> blobs, int blobCount) {
        telemetry.addLine("=== Artifact Detection Test ===");

        if (state == State.COMPLETE) {
            telemetry.addLine(String.format(Locale.US, "ALL %d TRIALS COMPLETE - %d passing",
                    TRIALS.length, trialsPassing));
            telemetry.addLine("-------------------");
            for (String line : sessionResults) {
                telemetry.addLine(line);
            }
            telemetry.addLine("-------------------");
            telemetry.addLine("Stop the OpMode. Both CSVs are written and flushed.");
            return;
        }

        telemetry.addData("TRIAL", "%d / %d", trialIndex + 1, TRIALS.length);
        telemetry.addData("CONDITION", condition());
        telemetry.addData("BALL", ">>> %s <<<", expectLabel().toUpperCase(Locale.US));

        switch (state) {
            case ARMED:
                telemetry.addData("STATUS", "ARMED - place the ball, press [A]");
                break;
            case RECORDING:
                telemetry.addData("STATUS", "RECORDING  %d / %d",
                        framesRecorded, Math.max(1, TARGET_FRAMES));
                if (System.currentTimeMillis() - lastFrameMillis > STALL_WARN_MS) {
                    telemetry.addLine("!! CAMERA STALLED - no new frames. [B] to redo.");
                }
                break;
            case DONE:
                telemetry.addData("STATUS", "DONE  %.1f%%  %s (target >= %.1f%%)",
                        passRate(), trialPassed() ? "PASS" : "FAIL", PASS_RATE_TARGET);
                telemetry.addData("Failures", "zero=%d  multi=%d  wrongLabel=%d",
                        zeroBlobFrames, multiBlobFrames, wrongLabelFrames);
                telemetry.addData("Mean aspect", "%.3f  (expect ~%.3f)",
                        safeMean(sumAspect, blobStatSamples), EXPECTED_ASPECT);
                telemetry.addLine("[A] commit + next    [B] discard + redo");
                break;
            default:
                break;
        }

        telemetry.addLine("-------------------");
        telemetry.addData("Session", "%d done, %d passing", trialIndex, trialsPassing);
        if (trialIndex + 1 < TRIALS.length) {
            telemetry.addData("NEXT", "%s (%s)", TRIALS[trialIndex + 1][0], TRIALS[trialIndex + 1][1]);
        } else {
            telemetry.addLine("NEXT: last trial of the session");
        }
        telemetry.addData("Snapshots", snapshotsOn ? "ON every " + SNAPSHOT_EVERY_N : "off");
        telemetry.addLine("[A]=start/commit  [B]=redo  [X]/[Y]=disable/enable");
        telemetry.addLine("[DPad Up]=toggle snapshots");

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
        telemetry.addData("Pipeline", artifacts.getDiagnosticSummary());
    }

    // -------------------------------------------------------------------------
    // Gate plumbing
    // -------------------------------------------------------------------------

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
}
