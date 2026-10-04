package org.firstinspires.ftc.teamcode.kalipsorobotics.test.cameraVision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision.AprilTagDetectionAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.AprilTagConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionManager;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionRecognition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.KAprilTagProcessor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.TagObservation;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.TagPoseFilter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblobbing.ArtifactColorBlobDetectionProcessor;

import java.util.List;
import java.util.Locale;

/**
 * Proves the point of the whole vision folder: ONE camera, MANY answers.
 *
 * The Arducam runs a color blob processor and an AprilTag processor on the same frames.
 * The blob side answers "where are the balls" for anything that wants them; the AprilTag
 * side is fed straight into AprilTagDetectionAction, which runs the robot -> turret ->
 * camera -> tag -> field transform chain and relocalizes into SharedData.
 *
 * WHAT TO LOOK FOR:
 *   TAGS      the tag IDs in view, and each one's camera-relative pose. camRelTag theta
 *             reads ~180 deg when the camera is square-on to the tag.
 *   RELOCALIZE the field position the transform chain produced, and WHICH TAG produced it.
 *             Any tag in the AprilTagConfig layout can relocalize the robot, and the
 *             nearest usable one wins, so "Fix from tag" changes as you drive. Walk the
 *             robot to a known spot and check the position against the tape measure. If it
 *             is mirrored or rotated, the field layout constants in AprilTagConfig are the
 *             suspect, not this file - and if only ONE tag reads wrong, it is that tag's
 *             layout entry specifically.
 *   BLOBS     detections from the same frames, to confirm neither processor is starving
 *             the other.
 *   FPS       the portal frame rate with both processors running. If it collapses, raise
 *             AprilTag decimation on the dashboard or disable a processor with [X]/[Y].
 *
 * Gamepad:
 *   [X]  disable the AprilTag processor (frees CPU, blobs keep running)
 *   [Y]  enable the AprilTag processor
 *   [A]  disable the blob processor
 *   [B]  enable the blob processor
 */
@Config
@TeleOp(name = "Test: Multi-Processor Vision", group = "Test Vision")
public class MultiProcessorVisionTest extends LinearOpMode {

    public static long EXPOSURE_MS = 20;
    public static int  GAIN        = 250;

    /** 1 sees far/small tags at full CPU cost, 2 is balanced, 3 is near-tags-only. */
    public static double APRILTAG_DECIMATION = 2;

    /** Set to BLUE to relocalize off tag 20 instead of tag 24. */
    public static AllianceColor ALLIANCE = AllianceColor.RED;

    /**
     * Relocalization needs the turret (its angle is a link in the transform chain). Turn
     * this off to run the OpMode on a bench with only a camera plugged in.
     */
    public static boolean RUN_RELOCALIZATION = true;

    private static final String TAG = "MultiProcessorVisionTest";

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        // ---- One camera, two processors -------------------------------------
        ArtifactColorBlobDetectionProcessor artifacts = new ArtifactColorBlobDetectionProcessor();
        KAprilTagProcessor aprilTags = new KAprilTagProcessor.Builder()
                .withDecimation((float) APRILTAG_DECIMATION)
                .build();

        VisionManager camera = new VisionManager.Builder(hardwareMap)
                .addProcessor(artifacts)
                .addProcessor(aprilTags)
                .streamImmediately()
                .build();

        camera.lockCameraControls(EXPOSURE_MS, GAIN);
        FtcDashboard.getInstance().startCameraStream(camera.getPortal(), 30);

        // ---- Hand the tag processor to the action that relocalizes off it ----
        int targetTagId = AprilTagConfig.getGoalAprilTagId(ALLIANCE);
        AprilTagDetectionAction relocalize = null;
        String relocalizeError = null;
        if (RUN_RELOCALIZATION) {
            try {
                Turret turret = Turret.getInstance(opModeUtilities);
                relocalize = new AprilTagDetectionAction(
                        opModeUtilities, turret, targetTagId, ALLIANCE, aprilTags);
            } catch (RuntimeException e) {
                // Bench runs with no turret wired up still get to see raw detections.
                relocalizeError = e.getMessage();
                KLog.e(TAG, "Relocalization disabled - turret unavailable", e);
            }
        }

        telemetry.addLine("=== Multi-Processor Vision Test ===");
        telemetry.addData("Processors", "%d on one portal", camera.getProcessors().size());
        telemetry.addData("Target tag", targetTagId);
        telemetry.addLine("Dashboard: http://192.168.43.1:8080");
        telemetry.update();

        waitForStart();

        double prevDecimation = APRILTAG_DECIMATION;

        while (opModeIsActive()) {
            if (APRILTAG_DECIMATION != prevDecimation) {
                aprilTags.setDecimation((float) APRILTAG_DECIMATION);
                prevDecimation = APRILTAG_DECIMATION;
            }

            if (gamepad1.xWasPressed()) camera.disable(aprilTags);
            if (gamepad1.yWasPressed()) camera.enable(aprilTags);
            if (gamepad1.aWasPressed()) camera.disable(artifacts);
            if (gamepad1.bWasPressed()) camera.enable(artifacts);

            if (relocalize != null) {
                relocalize.updateCheckDone();
            }

            renderTelemetry(camera, aprilTags, artifacts, relocalize, relocalizeError, targetTagId);
            telemetry.update();
        }

        FtcDashboard.getInstance().stopCameraStream();
        camera.close();
    }

    private void renderTelemetry(VisionManager camera,
                                 KAprilTagProcessor aprilTags,
                                 ArtifactColorBlobDetectionProcessor artifacts,
                                 AprilTagDetectionAction relocalize,
                                 String relocalizeError,
                                 int targetTagId) {

        telemetry.addData("FPS", "%.1f", camera.getPortal().getFps());
        telemetry.addLine("------- TAGS -------");
        List<TagObservation> observations = aprilTags.getLatestObservations();
        if (observations.isEmpty()) {
            telemetry.addLine("  none in view");
        } else {
            for (TagObservation observation : observations) {
                telemetry.addLine(String.format(Locale.US,
                        "  id=%d  camRelTag(x=%.0fmm y=%.0fmm th=%.1f deg)  tagRelCam(x=%.0f z=%.0f)",
                        observation.getTagId(),
                        observation.getCamRelTagPos().getX(),
                        observation.getCamRelTagPos().getY(),
                        Math.toDegrees(observation.getCamRelTagPos().getTheta()),
                        observation.getTagRelCamXMM(),
                        observation.getTagRelCamZMM()));
            }
        }
        telemetry.addData("AprilTag pipeline", aprilTags.getDiagnosticSummary());

        telemetry.addLine("------- RELOCALIZE -------");
        if (relocalizeError != null) {
            telemetry.addData("DISABLED", relocalizeError);
        } else if (relocalize == null) {
            telemetry.addLine("  RUN_RELOCALIZATION is off");
        } else {
            // Which tag actually produced the pose. With several tags in view the nearest
            // usable one wins, so this flips as the robot drives around the field.
            int fixTagId = relocalize.getLastFixTagId();
            telemetry.addData("Fix from tag",
                    fixTagId == AprilTagDetectionAction.NO_TAG ? "none" : String.valueOf(fixTagId));
            telemetry.addData("Field pos", SharedData.getLimelightGlobalPosition());
            telemetry.addData("Odometry", SharedData.getOdometryWheelIMUPosition());

            // Per-tag verdicts. A single tag reading REJECTED while the others are fine is
            // the signature of a wrong field pose for that tag in AprilTagConfig.
            for (int knownTagId : relocalize.getFieldLayout().getKnownTagIds()) {
                TagPoseFilter.Verdict verdict = relocalize.getVerdictFor(knownTagId);
                telemetry.addLine(String.format(Locale.US, "  tag %d: %s%s",
                        knownTagId,
                        verdict == null ? "never seen" : verdict,
                        knownTagId == targetTagId ? "   <- goal / aim tag" : ""));
            }

            telemetry.addData("Goal tag", "%d  %s",
                    targetTagId, aprilTags.isTagVisible(targetTagId) ? "IN VIEW" : "not seen");
            telemetry.addData("Raw to goal", SharedData.getLimelightRawPosition());
        }

        telemetry.addLine("------- BLOBS -------");
        List<VisionRecognition> blobs = artifacts.getLatestResult();
        if (blobs == null || blobs.isEmpty()) {
            telemetry.addLine("  none in view");
        } else {
            for (VisionRecognition blob : blobs) {
                telemetry.addLine(String.format(Locale.US,
                        "  %s a=%.0f c=%.2f px=(%.0f,%.0f)",
                        blob.label, blob.getArea(), blob.getCircularity(),
                        blob.center.getX(), blob.center.getY()));
            }
        }

        telemetry.addLine("[X]/[Y] tags off/on   [A]/[B] blobs off/on");
    }
}
