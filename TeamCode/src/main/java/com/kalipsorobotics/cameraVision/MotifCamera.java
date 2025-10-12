package com.kalipsorobotics.cameraVision;

import com.kalipsorobotics.modules.MotifColor;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public class MotifCamera {

    private static final int[] OBELISK_APRILTAG_IDS = {21, 22, 23};

    public static class MotifPattern {
        public final MotifColor top;
        public final MotifColor middle;
        public final MotifColor bottom;

        public MotifPattern(MotifColor top, MotifColor middle, MotifColor bottom) {
            this.top = top;
            this.middle = middle;
            this.bottom = bottom;
        }

        @Override
        public String toString() {
            return String.format("%s-%s-%s", top, middle, bottom);
        }

        public boolean equals(MotifColor expectedTop, MotifColor expectedMiddle, MotifColor expectedBottom) {
            return top == expectedTop && middle == expectedMiddle && bottom == expectedBottom;
        }
    }

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private MotifPattern cachedMotifPattern;
    private boolean motifPatternCached = false;
    WebcamName camera;

    public MotifCamera(OpModeUtilities opModeUtilities) {
        camera = opModeUtilities.getHardwareMap().get(WebcamName.class, "Webcam 1");
        this.aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                camera,
                aprilTagProcessor
        );
    }

    public WebcamName getCamera() {
        return camera;
    }
    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;
    }
    public VisionPortal getVisionPortal() {
        return visionPortal;
    }
}