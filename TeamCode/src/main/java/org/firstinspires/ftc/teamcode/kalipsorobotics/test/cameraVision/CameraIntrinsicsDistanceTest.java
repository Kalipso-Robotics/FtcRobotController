package org.firstinspires.ftc.teamcode.kalipsorobotics.test.cameraVision;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.vision.CameraIntrinsics.CAM_HEIGHT;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.vision.CameraIntrinsics.CAM_WIDTH;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Vector3d;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.TFLiteArtifactDetector;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionManager;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionRecognition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblob.ArtifactColorBlobDetectionProcessor;

/**
 * Live raytracing validation for CameraIntrinsics.calculateWorldPos /
 * getDistanceFromRobot. Distances are in mm.
 *
 * HOW TO VALIDATE THE RAYTRACING:
 *
 *   A) PROBE-PIXEL MODE (independent of the detector)
 *      Put a marker (piece of tape, sticker, anything visible) on the floor at
 *      a known distance from the robot. Open the dashboard camera stream and
 *      eyeball the pixel coords of where the marker touches the ground. Type
 *      those into PROBE_PX / PROBE_PY on the dashboard. The OpMode will print
 *      the world position and distance — compare against the tape-measure
 *      reading. Differences > ~3 cm usually mean mountAngle or cameraOffset is
 *      off in CameraIntrinsics.Arducam (lines 14-21).
 *
 *      The default probe pixel is (cx, cy + 0.4*frameHeight) which is roughly
 *      "centered, well below the horizon" — a safe starting point.
 *
 *   B) DETECTED-BLOB MODE (end-to-end check)
 *      Place a real artifact in view. The OpMode reports the projected floor
 *      position and distance for the largest purple and largest green blob.
 *      Compare those distances to a tape measure.
 *
 * VIEWING THE CAMERA STREAM:
 *   Dashboard at http://192.168.43.1:8080 → Camera tab. Color blob overlays
 *   are drawn by the processor. The probe pixel is *not* drawn (there is no
 *   draw hook for non-detector overlays in this stack) — read its world
 *   coordinate from telemetry.
 *
 * COORDINATE FRAME (CameraIntrinsics):
 *   Origin = robot center on the floor. +X = right, +Z (returned as Point.y) = forward.
 *   ROBOT_X / ROBOT_Y are where you tell the OpMode the robot currently is,
 *   for the distance computation. Leave at (0, 0) if testing from origin.
 */
@Config
@TeleOp
public class CameraIntrinsicsDistanceTest extends LinearOpMode {

    // ── Dashboard tunables ───────────────────────────────────────────────────
    /** Probe pixel — set to where your floor marker shows up in the frame. */
    public static double PROBE_PX = -1;   // negative = use default (cx, cy + 0.4*H)
    public static double PROBE_PY = -1;
    /** Pretend the robot is at this position when computing distance. */
    public static double ROBOT_X = 0.0;
    public static double ROBOT_Y = 0.0;
    /** Camera exposure / gain. Match what you used to tune the blob detector. */
    public static long EXPOSURE_MS = 20;
    public static int  GAIN        = 250;
    /** Camera resolution — must match the resolution used for intrinsic calibration. */

    // ── Mount-geometry tunables (this is what you're hunting for) ────────────
    /** Camera tilt below horizontal, in degrees. Positive = lens points downward. */
    public static double MOUNT_ANGLE_DEG = 0.0;
    /** Camera height above floor in mm (Y component of cameraOffset). */
    public static double CAM_HEIGHT_MM   = 236.163;
    /** Lateral offset of camera from robot center in mm (X). +X = right. */
    public static double CAM_OFFSET_X_MM = 157.548;
    /** Forward offset of camera from robot center in mm (Z). +Z = forward. */
    public static double CAM_OFFSET_Z_MM = 151.868;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        ArtifactColorBlobDetectionProcessor artifacts = new ArtifactColorBlobDetectionProcessor();

        VisionManager visionManager = new VisionManager.Builder(hardwareMap)
                .withResolution(CAM_WIDTH, CAM_HEIGHT)
                .addProcessor(artifacts)
                .streamImmediately()
                .build();

        visionManager.lockCameraControls(EXPOSURE_MS, GAIN);
        FtcDashboard.getInstance().startCameraStream(visionManager.getPortal(), 30);

        long prevExposure = EXPOSURE_MS;
        int  prevGain     = GAIN;

        telemetry.addLine("Ready. Press PLAY to start raytrace test.");
        telemetry.addLine("Tune MOUNT_ANGLE_DEG and CAM_HEIGHT_MM until probe distance");
        telemetry.addLine("matches your tape-measured ground-truth.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (EXPOSURE_MS != prevExposure || GAIN != prevGain) {
                visionManager.lockCameraControls(EXPOSURE_MS, GAIN);
                prevExposure = EXPOSURE_MS;
                prevGain     = GAIN;
            }

            // Rebuild intrinsics each loop so dashboard tweaks take effect live.
            CameraIntrinsics intrinsics = CameraIntrinsics.ARDUCAM.withMount(
                    Math.toRadians(MOUNT_ANGLE_DEG),
                    new Vector3d(CAM_OFFSET_X_MM, CAM_HEIGHT_MM, CAM_OFFSET_Z_MM));

            Point robotPos = new Point(ROBOT_X, ROBOT_Y);

            // ─── A. Probe-pixel raytrace (detector-independent) ──────────────
            double px = (PROBE_PX < 0) ? intrinsics.getCx()                        : PROBE_PX;
            double py = (PROBE_PY < 0) ? intrinsics.getCy() + 0.4 * CAM_HEIGHT      : PROBE_PY;

            telemetry.addData("Image WxH", "%d x %d", CAM_WIDTH, CAM_HEIGHT);
            telemetry.addData("cx, cy",    "(%.2f, %.2f)", intrinsics.getCx(), intrinsics.getCy());
            telemetry.addData("Mount angle (deg)", "%.2f", MOUNT_ANGLE_DEG);
            telemetry.addData("Cam offset (mm) X,Y,Z", "(%.1f, %.1f, %.1f)",
                    CAM_OFFSET_X_MM, CAM_HEIGHT_MM, CAM_OFFSET_Z_MM);
            telemetry.addData("Robot pos (mm)", "(%.1f, %.1f)", ROBOT_X, ROBOT_Y);
            telemetry.addLine("─── PROBE PIXEL ───");
            dumpRayTrace("PROBE", px, py, robotPos, intrinsics);

            // ─── B. TFLite detections (end-to-end check) ────────────────────
            telemetry.addLine("─── TFLITE DETECTIONS ───");
            List<VisionRecognition> recognitions = artifacts.getLatestResult();
            if (recognitions == null || recognitions.isEmpty()) {
                telemetry.addLine("No artifacts detected.");
            } else {
                telemetry.addData("Detection count", recognitions.size());
                VisionRecognition largest = recognitions.get(0);
                for (VisionRecognition r : recognitions) {
                    if (r.getArea() > largest.getArea()) largest = r;
                }
                reportRecognitionWithDump("LARGEST", largest, intrinsics, robotPos);
            }

            telemetry.update();
            sleep(50);
        }

        FtcDashboard.getInstance().stopCameraStream();
        visionManager.close();
    }

    private void reportRecognitionWithDump(String name, VisionRecognition r,
                                           CameraIntrinsics intrinsics, Point robotPos) {
        if (r == null) {
            telemetry.addData(name, "no detection");
            return;
        }
        telemetry.addData(name + " label",     r.formattedLabel);
        telemetry.addData(name + " box L,T,R,B",
                "(%.0f, %.0f, %.0f, %.0f)", r.left, r.top, r.right, r.bottom);
        Point pixel = r.getBottomMiddlePixel();
        dumpRayTrace(name, pixel.getX(), pixel.getY(), robotPos, intrinsics);
    }

    /**
     * Recomputes every intermediate value in calculateWorldPos and logs them to
     * both telemetry and logcat (tag = "RAYTRACE_DBG"). Pull with:
     *   adb logcat -s RAYTRACE_DBG
     */
    private void dumpRayTrace(String label, double pixelX, double pixelY,
                              Point robotPos, CameraIntrinsics intrinsics) {
        double cx = intrinsics.getCx();
        double cy = intrinsics.getCy();
        // We can't read fx/fy/focalLength from the intrinsics without a getter,
        // so we mirror the math here using the same constants as CameraIntrinsics.ARDUCAM
        // (1280x800 calibration scaled to 640x480 — see CameraIntrinsics.java).
        double fx = 444.14195;
        double fy = 532.27560;
        double focal = (fx + fy) / 2.0;
        double mountRad = Math.toRadians(MOUNT_ANGLE_DEG);

        double normX = pixelX - cx;
        // Match CameraIntrinsics.calculateWorldPos: image-Y is flipped to world Y-up.
        double normY = cy - pixelY;
        double xDir = normX / focal;
        double yDir = normY / focal;
        double zDir = 1.0;

        double worldX = xDir;
        double worldY = yDir * Math.cos(mountRad) - zDir * Math.sin(mountRad);
        double worldZ = yDir * Math.sin(mountRad) + zDir * Math.cos(mountRad);

        boolean rejected = worldY > 1e-6;
        double t = rejected ? Double.NaN : -CAM_HEIGHT_MM / worldY;
        double floorX = rejected ? Double.NaN : t * worldX;
        double floorZ = rejected ? Double.NaN : t * worldZ;
        double xFromRobot = rejected ? Double.NaN : floorX + CAM_OFFSET_X_MM;
        double zFromRobot = rejected ? Double.NaN : floorZ + CAM_OFFSET_Z_MM;

        // Also call the real method so we know we match.
        Point realWorld = intrinsics.calculateWorldPos(pixelX, pixelY);
        double realDist = intrinsics.getDistanceFromRobot(pixelX, pixelY,
                robotPos.getX(), robotPos.getY());

        String dump = String.format(Locale.US,
                "[%s] px=(%.2f,%.2f) cxy=(%.2f,%.2f) focal=%.2f mountDeg=%.3f " +
                "norm=(%.3f,%.3f) dir=(%.5f,%.5f,%.5f) world=(%.5f,%.5f,%.5f) " +
                "rejected=%b t=%.2f floor=(%.2f,%.2f) fromRobot=(%.2f,%.2f) " +
                "realWorld=%s realDist=%.2f",
                label, pixelX, pixelY, cx, cy, focal, MOUNT_ANGLE_DEG,
                normX, normY, xDir, yDir, zDir, worldX, worldY, worldZ,
                rejected, t, floorX, floorZ, xFromRobot, zFromRobot,
                realWorld == null ? "null"
                        : String.format(Locale.US, "(%.2f,%.2f)", realWorld.getX(), realWorld.getY()),
                realDist);

        Log.i("RAYTRACE_DBG", dump);
        telemetry.addLine(dump);
    }
}
