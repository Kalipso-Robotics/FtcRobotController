package org.firstinspires.ftc.teamcode.kalipsorobotics.tensorflow;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

public class CameraCapture {
    /*
     * EDIT THESE PARAMETERS AS NEEDED
     */
    final boolean USING_WEBCAM = false;
    final BuiltinCameraDirection INTERNAL_CAM_DIR = BuiltinCameraDirection.BACK;
    final int RESOLUTION_WIDTH = 640;
    final int RESOLUTION_HEIGHT = 480;

    // Internal state
    boolean lastX;
    int frameCount;
    long capReqTime = 0;
    VisionPortal portal;

    public CameraCapture() {
        portal = new VisionPortal.Builder()
                .setCamera(INTERNAL_CAM_DIR)
                .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                .build();
        capReqTime = 0;
    }

    public void capture() {
        if (capReqTime == 0) {
            portal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d", frameCount++));
            KLog.d("Camera TF", "writing to CameraFrameCapture. frame count: " + frameCount);
            capReqTime = System.currentTimeMillis();
        }

        if (capReqTime != 0 && System.currentTimeMillis() - capReqTime > 500)
        {
            capReqTime = 0;
        }
    }
}
