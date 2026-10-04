package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

/**
 * Saves raw camera frames to disk for building training datasets.
 *
 * Frames are written to /sdcard/VisionPortal-<PREFIX>_<n>.png on the Control Hub.
 * Pull them off with: adb pull /sdcard/ (or Android Studio's Device Explorer).
 *
 * NOTE: change PREFIX between runs. Numbering restarts at 0 every run, so a
 * second run with the same prefix overwrites the first run's frames.
 */
@TeleOp(name = "Capture Frames", group = "Data")
public class CaptureFrames extends LinearOpMode {

    private static final String PREFIX = "clip1";
    private static final long INTERVAL_MS = 700;

    @Override
    public void runOpMode() {
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // Wait for the camera to come up before allowing start - a capture
        // requested while not streaming is silently dropped.
        while (!isStarted() && !isStopRequested()) {
            VisionPortal.CameraState state = portal.getCameraState();
            telemetry.addData("camera", state);
            if (state == VisionPortal.CameraState.STREAMING) {
                telemetry.addLine("camera ready - press start");
            } else if (state == VisionPortal.CameraState.ERROR) {
                telemetry.addLine("CAMERA ERROR - check config name / usb cable");
            } else {
                telemetry.addLine("waiting for camera");
            }
            telemetry.update();
            sleep(50);
        }

        int n = 0;
        long last = 0;
        while (opModeIsActive()) {
            if (System.currentTimeMillis() - last > INTERVAL_MS) {
                portal.saveNextFrameRaw(String.format(Locale.US, "%s_%04d", PREFIX, n));
                n++;
                last = System.currentTimeMillis();
            }
            telemetry.addData("frames saved", n);
            telemetry.addData("prefix", PREFIX);
            telemetry.addData("path", "/sdcard/VisionPortal-" + PREFIX + "_XXXX.png");
            telemetry.update();
            sleep(20);
        }

        portal.close();
    }
}
