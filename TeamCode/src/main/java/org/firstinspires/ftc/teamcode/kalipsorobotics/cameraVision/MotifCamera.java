package org.firstinspires.ftc.teamcode.kalipsorobotics.cameraVision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.MotifColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class MotifCamera {

    private static final int[] OBELISK_APRILTAG_IDS = {21, 22, 23};

    public static class MotifPattern {
        public final MotifColor first;
        public final MotifColor middle;
        public final MotifColor last;

        public MotifPattern(MotifColor first, MotifColor middle, MotifColor last) {
            this.first = first;
            this.middle = middle;
            this.last = last;
        }

        @Override
        public String toString() {
            return String.format("%s-%s-%s", first, middle, last);
        }

        public boolean equals(MotifColor expectedFirst, MotifColor expectedMiddle, MotifColor expectedLast) {
            return first == expectedFirst && middle == expectedMiddle && last == expectedLast;
        }
    }

    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private MotifPattern cachedMotifPattern;
    private final boolean motifPatternCached = false;
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