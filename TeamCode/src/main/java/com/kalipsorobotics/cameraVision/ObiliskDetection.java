package com.kalipsorobotics.cameraVision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.ArrayList;

public class ObiliskDetection {
    
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
    
    public ObiliskDetection() {
    }
    
    public void init(HardwareMap hardwareMap) {
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        
        visionPortal = VisionPortal.easyCreateWithDefaults(
            hardwareMap.get(WebcamName.class, "Webcam 1"),
            aprilTagProcessor
        );
    }
    
    public List<AprilTagDetection> getObeliskDetections() {
        List<AprilTagDetection> allDetections = aprilTagProcessor.getDetections();
        return allDetections.stream()
            .filter(detection -> isObeliskTag(detection.id))
            .collect(java.util.stream.Collectors.toList());
    }
    
    public AprilTagDetection getClosestObeliskDetection() {
        List<AprilTagDetection> obeliskDetections = getObeliskDetections();
        
        if (obeliskDetections.isEmpty()) {
            return null;
        }
        
        return obeliskDetections.stream()
            .filter(d -> d.ftcPose != null)
            .min((d1, d2) -> Double.compare(d1.ftcPose.range, d2.ftcPose.range))
            .orElse(null);
    }
    
    public double getDistanceToObelisk() {
        AprilTagDetection closest = getClosestObeliskDetection();
        return closest != null && closest.ftcPose != null ? closest.ftcPose.range : -1;
    }
    
    public double getBearingToObelisk() {
        AprilTagDetection closest = getClosestObeliskDetection();
        return closest != null && closest.ftcPose != null ? closest.ftcPose.bearing : 0;
    }
    
    public double getElevationToObelisk() {
        AprilTagDetection closest = getClosestObeliskDetection();
        return closest != null && closest.ftcPose != null ? closest.ftcPose.elevation : 0;
    }
    
    public boolean isObeliskVisible() {
        return !getObeliskDetections().isEmpty();
    }
    
    private boolean isObeliskTag(int tagId) {
        for (int id : OBELISK_APRILTAG_IDS) {
            if (id == tagId) {
                return true;
            }
        }
        return false;
    }
    
    public MotifPattern getObeliskMotifPattern() {
        if (!motifPatternCached) {
            updateMotifPattern();
        }
        return cachedMotifPattern;
    }
    
    public void updateMotifPattern() {
        AprilTagDetection obeliskDetection = getClosestObeliskDetection();
        if (obeliskDetection == null) {
            cachedMotifPattern = new MotifPattern(MotifColor.UNKNOWN, MotifColor.UNKNOWN, MotifColor.UNKNOWN);
        } else {
            // Return expected pattern based on AprilTag ID since image analysis isn't working
            cachedMotifPattern = getExpectedMotifPattern(obeliskDetection.id);
        }
        motifPatternCached = true;
    }
    
    public void refreshMotifPattern() {
        motifPatternCached = false;
        updateMotifPattern();
    }
    
    public MotifPattern getExpectedMotifPattern(int aprilTagId) {
        switch (aprilTagId) {
            case 21: return new MotifPattern(MotifColor.GREEN, MotifColor.PURPLE, MotifColor.PURPLE);
            case 22: return new MotifPattern(MotifColor.PURPLE, MotifColor.GREEN, MotifColor.PURPLE);
            case 23: return new MotifPattern(MotifColor.PURPLE, MotifColor.PURPLE, MotifColor.GREEN);
            default: return new MotifPattern(MotifColor.UNKNOWN, MotifColor.UNKNOWN, MotifColor.UNKNOWN);
        }
    }
    
    public int getObeliskId() {
        List<AprilTagDetection> obeliskDetections = getObeliskDetections();
        if (obeliskDetections.isEmpty()) {
            return -1;
        }
        
        // Return the first obelisk detection's ID, regardless of pose data
        return obeliskDetections.get(0).id;
    }
    
    public MotifPattern getExpectedMotifPattern() {
        int id = getObeliskId();
        return getExpectedMotifPattern(id);
    }
    
    private MotifPattern analyzeMotifPattern(AprilTagDetection detection) {
        Mat frame = visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING ? 
            null : getCurrentFrame();
            
        if (frame == null) {
            return new MotifPattern(MotifColor.UNKNOWN, MotifColor.UNKNOWN, MotifColor.UNKNOWN);
        }
        
        int centerX = (int) detection.center.x;
        int centerY = (int) detection.center.y;
        
        int regionSize = 30;
        int topY = Math.max(0, centerY - 60);
        int middleY = centerY;
        int bottomY = Math.min(frame.rows() - regionSize, centerY + 60);
        int leftX = Math.max(0, centerX - regionSize/2);
        int rightX = Math.min(frame.cols(), centerX + regionSize/2);
        
        MotifColor topColor = detectColorInRegion(frame, leftX, topY, rightX - leftX, regionSize);
        MotifColor middleColor = detectColorInRegion(frame, leftX, middleY, rightX - leftX, regionSize);
        MotifColor bottomColor = detectColorInRegion(frame, leftX, bottomY, rightX - leftX, regionSize);
        
        return new MotifPattern(topColor, middleColor, bottomColor);
    }
    
    private MotifColor detectColorInRegion(Mat frame, int x, int y, int width, int height) {
        if (x < 0 || y < 0 || x + width > frame.cols() || y + height > frame.rows()) {
            return MotifColor.UNKNOWN;
        }
        
        Rect region = new Rect(x, y, width, height);
        Mat roi = new Mat(frame, region);
        Mat hsv = new Mat();
        Imgproc.cvtColor(roi, hsv, Imgproc.COLOR_RGB2HSV);
        
        Scalar purpleLower = new Scalar(130, 50, 50);
        Scalar purpleUpper = new Scalar(170, 255, 255);
        Scalar greenLower = new Scalar(35, 50, 50);
        Scalar greenUpper = new Scalar(85, 255, 255);
        
        Mat purpleMask = new Mat();
        Mat greenMask = new Mat();
        
        Core.inRange(hsv, purpleLower, purpleUpper, purpleMask);
        Core.inRange(hsv, greenLower, greenUpper, greenMask);
        
        double purplePixels = Core.sumElems(purpleMask).val[0] / 255.0;
        double greenPixels = Core.sumElems(greenMask).val[0] / 255.0;
        
        roi.release();
        hsv.release();
        purpleMask.release();
        greenMask.release();
        
        if (purplePixels > greenPixels && purplePixels > 100) {
            return MotifColor.PURPLE;
        } else if (greenPixels > 100) {
            return MotifColor.GREEN;
        } else {
            return MotifColor.UNKNOWN;
        }
    }
    
    private Mat getCurrentFrame() {
        return null;
    }
    
    public boolean isMotifPattern(MotifColor expectedTop, MotifColor expectedMiddle, MotifColor expectedBottom) {
        MotifPattern pattern = getObeliskMotifPattern();
        return pattern.equals(expectedTop, expectedMiddle, expectedBottom);
    }
    
    public String getMotifPatternString() {
        MotifPattern pattern = getObeliskMotifPattern();
        return pattern.toString();
    }
    
    public void addTelemetry(Telemetry telemetry) {
        List<AprilTagDetection> obeliskDetections = getObeliskDetections();
        telemetry.addData("Obelisk Tags Detected", obeliskDetections.size());
        
        if (!obeliskDetections.isEmpty()) {
            int obeliskId = getObeliskId();
            telemetry.addData("Obelisk ID", obeliskId);
            
            MotifPattern detectedPattern = getObeliskMotifPattern();
            MotifPattern expectedPattern = getExpectedMotifPattern(obeliskId);
            
            telemetry.addData("Detected Pattern", detectedPattern.toString());
            telemetry.addData("Expected Pattern", expectedPattern.toString());
            
            String patternName = getPatternName(obeliskId);
            if (!patternName.isEmpty()) {
                telemetry.addData("Pattern Name", patternName);
            }
        }
        
        for (AprilTagDetection detection : obeliskDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== Obelisk (ID %d) %s", 
                    detection.id, detection.metadata.name));
                if (detection.ftcPose != null) {
                    telemetry.addLine(String.format("Distance: %.1f inches", detection.ftcPose.range));
                    telemetry.addLine(String.format("Bearing: %.1f degrees", detection.ftcPose.bearing));
                    telemetry.addLine(String.format("Elevation: %.1f degrees", detection.ftcPose.elevation));
                    telemetry.addLine(String.format("Position XYZ: %.1f %.1f %.1f", 
                        detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                } else {
                    telemetry.addLine("Pose data unavailable");
                }
            } else {
                telemetry.addLine(String.format("\n==== Unknown Obelisk Tag (ID %d)", detection.id));
                if (detection.ftcPose != null) {
                    telemetry.addLine(String.format("Distance: %.1f inches", detection.ftcPose.range));
                } else {
                    telemetry.addLine("Pose data unavailable");
                }
            }
        }
    }
    
    private String getPatternName(int aprilTagId) {
        switch (aprilTagId) {
            case 21: return "Green-Purple-Purple 🟩🟪🟪";
            case 22: return "Purple-Green-Purple 🟪🟩🟪";
            case 23: return "Purple-Purple-Green 🟪🟪🟩";
            default: return "";
        }
    }
    
    public void stopStreaming() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }
    
    public void resumeStreaming() {
        if (visionPortal != null) {
            visionPortal.resumeStreaming();
        }
    }
    
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
