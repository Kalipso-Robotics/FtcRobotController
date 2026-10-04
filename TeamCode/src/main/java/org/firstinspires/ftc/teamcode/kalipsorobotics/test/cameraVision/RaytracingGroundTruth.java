package org.firstinspires.ftc.teamcode.kalipsorobotics.test.cameraVision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.BallInformation;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.Raytracer;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.TFLitePollenNectarDetector;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionManager;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionRecognition;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Ground-truth collector for the production ball pipeline: TFLite detector -> Raytracer ->
 * SharedData, at the stream size VisionConfig.ARDUCAM is calibrated for. What lands in
 * SharedData.getBallInformation() is exactly what pathing and selection code will see.
 *
 * Follow the SHOT LIST on telemetry. Keep the robot still, put a ball at the tape-measured spot
 * FROM ROBOT CENTRE, pull LT to capture a burst, Y to save it (and move to the next shot), A to
 * discard. The robot is pinned to the origin, so field x = forward and field y = left, and the
 * error columns are directly "published minus tape".
 *
 * The shot list is 5 distances x 3 lateral offsets. The centred column fits the mount pitch;
 * the +/-12 in columns are what make yaw, roll and the camera x offset identifiable. Then run
 * RaytracerFitTest on the CSV (see its javadoc).
 *
 * The mount comes from VisionConfig.ARDUCAM.mount and is written on every row, so a CSV is
 * self-describing: it can be re-fit against whatever mount it was collected with.
 *
 * Controls: LT burst, Y save + next shot, A discard, DPad R/L next/previous shot.
 */
@TeleOp(name = "Raytracing Ground Truth", group = "Test")
public class RaytracingGroundTruth extends LinearOpMode {

    private static final String TAG = "RaytracingGT";
    private static final double MM_PER_INCH = 25.4;
    private static final int BURST_FRAMES = 30;

    /** Inches forward of robot centre. */
    private static final double[] DISTANCES_IN = {20, 30, 40, 50, 60};
    /** Inches left of robot centre (negative = right). */
    private static final double[] LATERALS_IN = {0, -12, 12};

    private static final String CSV_HEADER =
            "KnownDistMM,KnownLateralMM,Label,Confidence,"
            + "EdgeL,EdgeR,EdgeT,EdgeB,"
            + "FieldX,FieldY,DistMM,FwdErrMM,LatErrMM,DistErrMM,"
            + "RayDepthMM,SizeVDepthMM,SizeHDepthMM,Consistent,"
            + "PitchDeg,YawDeg,RollDeg,CamXMM,CamYMM,CamZMM,CaptureNanos";

    private final List<String> burst = new ArrayList<>();
    private boolean capturing = false;
    private boolean wasLeftTriggerPressed = false;
    private long lastCapturedNanos = -1;
    private int savedRows = 0;
    private int shot = 0;
    private final boolean[] shotDone = new boolean[DISTANCES_IN.length * LATERALS_IN.length];

    private static double distanceMM(int shot) {
        return DISTANCES_IN[shot % DISTANCES_IN.length] * MM_PER_INCH;
    }

    private static double lateralMM(int shot) {
        return LATERALS_IN[shot / DISTANCES_IN.length] * MM_PER_INCH;
    }

    private static String describe(int shot) {
        double lat = LATERALS_IN[shot / DISTANCES_IN.length];
        return String.format(Locale.US, "%.0f in forward, %s", DISTANCES_IN[shot % DISTANCES_IN.length],
                lat == 0 ? "centred" : String.format(Locale.US, "%.0f in %s", Math.abs(lat), lat > 0 ? "LEFT" : "RIGHT"));
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        KFileWriter fileWriter = new KFileWriter("RaytracingGroundTruth", opModeUtilities);
        fileWriter.writeLine(CSV_HEADER);

        VisionConfig.Camera camera = VisionConfig.ARDUCAM;
        SharedData.resetBallInformation();
        SharedData.setOdometryWheelIMUPosition(new Position(0, 0, 0));

        TFLitePollenNectarDetector detector = new TFLitePollenNectarDetector(hardwareMap.appContext)
                .withMinConfidence(0.35f);
        Raytracer raytracer = new Raytracer(detector, camera);

        VisionManager visionManager = new VisionManager.Builder(hardwareMap)
                .withCamera(camera)
                .addProcessor(detector)
                .streamImmediately()
                .build();
        FtcDashboard.getInstance().startCameraStream(visionManager.getPortal(), 30);

        telemetry.addLine("=== Raytracing Ground Truth ===");
        telemetry.addData("Writing to", fileWriter.getPath());
        telemetry.addLine("Press PLAY to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Pinned at the origin so published field coords == robot-relative coords.
            SharedData.setOdometryWheelIMUPosition(new Position(0, 0, 0));

            if (gamepad1.dpadRightWasPressed()) shot = (shot + 1) % shotDone.length;
            if (gamepad1.dpadLeftWasPressed()) shot = (shot + shotDone.length - 1) % shotDone.length;
            double knownDistanceMM = distanceMM(shot), knownLateralMM = lateralMM(shot);

            // The published ball nearest the target spot, with the box it came from.
            Raytracer.Result r = raytracer.getLastResult();
            int best = -1;
            double bestGap = Double.MAX_VALUE;
            for (int i = 0; i < r.balls.size(); i++) {
                BallInformation b = r.balls.get(i);
                double gap = Math.hypot(b.x - knownDistanceMM, b.y - knownLateralMM);
                if (gap < bestGap) {
                    bestGap = gap;
                    best = i;
                }
            }

            boolean triggerDown = gamepad1.left_trigger > 0.1;
            if (triggerDown && !wasLeftTriggerPressed) {
                burst.clear();
                capturing = true;
                lastCapturedNanos = -1;
            }
            wasLeftTriggerPressed = triggerDown;

            telemetry.addData("SHOT", "%d/%d  put the ball %s", shot + 1, shotDone.length, describe(shot));
            if (best >= 0) {
                BallInformation b = r.balls.get(best);
                VisionRecognition d = r.detections.get(best);
                Raytracer.Estimate e = r.estimates.get(best);
                if (capturing && r.captureNanos != lastCapturedNanos) {
                    lastCapturedNanos = r.captureNanos;
                    burst.add(csvRow(b, d, e, knownDistanceMM, knownLateralMM, r.captureNanos));
                    if (burst.size() >= BURST_FRAMES) capturing = false;
                }
                telemetry.addLine(String.format(Locale.US,
                        "%s %.0f%%  field=(%.0f, %.0f)  err fwd %+.0f lat %+.0f mm",
                        b.type, b.confidence * 100, b.x, b.y,
                        b.x - knownDistanceMM, b.y - knownLateralMM));
                telemetry.addLine(String.format(Locale.US, "ray depth %.0f  sizeV %.0f  sizeH %.0f  %s",
                        e.rayDepthMM, e.sizeDepthVMM, e.sizeDepthHMM,
                        e.isConsistent() ? "consistent" : "INCONSISTENT"));
            } else {
                telemetry.addLine("No ball published.");
            }

            if (gamepad1.aWasPressed()) {
                burst.clear();
                capturing = false;
            }
            if (gamepad1.yWasPressed() && !burst.isEmpty()) {
                for (String row : burst) {
                    fileWriter.writeLine(row);
                    savedRows++;
                }
                try {
                    fileWriter.flush();
                } catch (IOException ex) {
                    KLog.e(TAG, "Failed to flush burst", ex);
                }
                burst.clear();
                shotDone[shot] = true;
                for (int i = 1; i <= shotDone.length; i++) { // next shot not yet done
                    int next = (shot + i) % shotDone.length;
                    if (!shotDone[next]) { shot = next; break; }
                }
            }

            int done = 0;
            for (boolean b : shotDone) if (b) done++;
            telemetry.addData("Done", "%d/%d shots %s", done, shotDone.length,
                    shotDone[shot] ? "(this one already saved)" : "");
            telemetry.addData("Burst", "%d/%d %s  rows=%d", burst.size(), BURST_FRAMES,
                    capturing ? "CAPTURING" : "", savedRows);
            telemetry.addData("Model", detector.getDiagnosticSummary());
            telemetry.addLine("LT: capture   Y: save+next   A: discard   DPad L/R: shot");
            telemetry.update();
        }

        String path = fileWriter.getPath();
        fileWriter.close();
        FtcDashboard.getInstance().stopCameraStream();
        visionManager.close();
        KLog.d(TAG, "Done. " + savedRows + " rows in " + path);
    }

    private String csvRow(BallInformation b, VisionRecognition d, Raytracer.Estimate e,
                          double knownDistanceMM, double knownLateralMM, long captureNanos) {
        VisionConfig.Mount m = VisionConfig.ARDUCAM.mount;
        return String.format(Locale.US,
                "%.1f,%.1f,%s,%.3f,"
                + "%.1f,%.1f,%.1f,%.1f,"
                + "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
                + "%.2f,%.2f,%.2f,%d,"
                + "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d",
                knownDistanceMM, knownLateralMM, d.label, d.confidence,
                d.left, d.right, d.top, d.bottom,
                b.x, b.y, b.distanceMM, b.x - knownDistanceMM, b.y - knownLateralMM,
                b.distanceMM - Math.hypot(knownDistanceMM, knownLateralMM),
                e.rayDepthMM, e.sizeDepthVMM, e.sizeDepthHMM, e.isConsistent() ? 1 : 0,
                m.pitchDeg, m.yawDeg, m.rollDeg, m.xMM, m.yMM, m.zMM, captureNanos);
    }
}
