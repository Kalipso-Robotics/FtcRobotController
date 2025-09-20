package com.kalipsorobotics.cameraVision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class GoalDetection {
    
    private static final int[] GOAL_APRILTAG_IDS = {24, 25};
    
    public static class GoalInfo {
        public final AllianceColor color;
        public final double distance;
        public final double bearing;
        public final double elevation;
        public final int aprilTagId;
        
        public GoalInfo(AllianceColor color, double distance, double bearing, double elevation, int aprilTagId) {
            this.color = color;
            this.distance = distance;
            this.bearing = bearing;
            this.elevation = elevation;
            this.aprilTagId = aprilTagId;
        }
        
        @Override
        public String toString() {
            return String.format("%s Goal (ID %d) - Distance: %.1f inches", color, aprilTagId, distance);
        }
    }
    
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    
    public GoalDetection() {
    }
    
    public void init(HardwareMap hardwareMap) {
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        
        visionPortal = VisionPortal.easyCreateWithDefaults(
            hardwareMap.get(WebcamName.class, "Webcam 1"),
            aprilTagProcessor
        );
    }
    
    public List<AprilTagDetection> getGoalDetections() {
        List<AprilTagDetection> allDetections = aprilTagProcessor.getDetections();
        return allDetections.stream()
            .filter(detection -> isGoalTag(detection.id))
            .collect(java.util.stream.Collectors.toList());
    }
    
    public AprilTagDetection getClosestGoalDetection() {
        List<AprilTagDetection> goalDetections = getGoalDetections();
        
        if (goalDetections.isEmpty()) {
            return null;
        }
        
        return goalDetections.stream()
            .filter(d -> d.ftcPose != null)
            .min((d1, d2) -> Double.compare(d1.ftcPose.range, d2.ftcPose.range))
            .orElse(null);
    }
    
    public GoalInfo getClosestGoalInfo() {
        AprilTagDetection closest = getClosestGoalDetection();
        
        if (closest == null || closest.ftcPose == null) {
            return null;
        }
        
        AllianceColor color = getGoalColor(closest.id);
        return new GoalInfo(color, closest.ftcPose.range, closest.ftcPose.bearing, 
                           closest.ftcPose.elevation, closest.id);
    }
    
    public AllianceColor getGoalColor(int aprilTagId) {
        switch (aprilTagId) {
            case 24: return AllianceColor.RED;
            case 25: return AllianceColor.BLUE;
            default: return null;
        }
    }
    
    public double getDistanceToGoal() {
        AprilTagDetection closest = getClosestGoalDetection();
        return closest != null && closest.ftcPose != null ? closest.ftcPose.range : -1;
    }
    
    public double getBearingToGoal() {
        AprilTagDetection closest = getClosestGoalDetection();
        return closest != null && closest.ftcPose != null ? closest.ftcPose.bearing : 0;
    }
    
    public double getElevationToGoal() {
        AprilTagDetection closest = getClosestGoalDetection();
        return closest != null && closest.ftcPose != null ? closest.ftcPose.elevation : 0;
    }
    
    public double getHeadingToGoal() {
        AprilTagDetection closest = getClosestGoalDetection();
        return closest != null && closest.ftcPose != null ? closest.ftcPose.bearing : 0;
    }
    
    public double getHeadingToGoal(double currentHeading) {
        double bearing = getHeadingToGoal();
        double targetHeading = currentHeading + bearing;
        
        while (targetHeading > 180) targetHeading -= 360;
        while (targetHeading <= -180) targetHeading += 360;
        
        return targetHeading;
    }
    
    public double getHeadingError(double currentHeading) {
        double targetHeading = getHeadingToGoal(currentHeading);
        double error = targetHeading - currentHeading;
        
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;
        
        return error;
    }
    
    public boolean isGoalVisible() {
        return !getGoalDetections().isEmpty();
    }
    
    public boolean isRedGoalVisible() {
        return getGoalDetections().stream()
            .anyMatch(detection -> detection.id == 24);
    }
    
    public boolean isBlueGoalVisible() {
        return getGoalDetections().stream()
            .anyMatch(detection -> detection.id == 25);
    }
    
    private boolean isGoalTag(int tagId) {
        for (int id : GOAL_APRILTAG_IDS) {
            if (id == tagId) {
                return true;
            }
        }
        return false;
    }
    
    public int getGoalId() {
        List<AprilTagDetection> goalDetections = getGoalDetections();
        if (goalDetections.isEmpty()) {
            return -1;
        }
        
        return goalDetections.get(0).id;
    }
    
    public void addTelemetry(Telemetry telemetry) {
        List<AprilTagDetection> goalDetections = getGoalDetections();
        telemetry.addData("Goal Tags Detected", goalDetections.size());
        
        GoalInfo goalInfo = getClosestGoalInfo();
        if (goalInfo != null) {
            telemetry.addData("Closest Goal", goalInfo.color.toString());
            telemetry.addData("Goal Distance", String.format("%.1f inches", goalInfo.distance));
            telemetry.addData("Goal Bearing", String.format("%.1f degrees", goalInfo.bearing));
            telemetry.addData("Goal Elevation", String.format("%.1f degrees", goalInfo.elevation));
        }
        
        for (AprilTagDetection detection : goalDetections) {
            AllianceColor color = getGoalColor(detection.id);
            telemetry.addLine(String.format("\n==== %s Goal (ID %d)", 
                color != null ? color.toString() : "UNKNOWN", detection.id));
            
            if (detection.ftcPose != null) {
                telemetry.addLine(String.format("Distance: %.1f inches", detection.ftcPose.range));
                telemetry.addLine(String.format("Bearing: %.1f degrees", detection.ftcPose.bearing));
                telemetry.addLine(String.format("Elevation: %.1f degrees", detection.ftcPose.elevation));
                telemetry.addLine(String.format("Position XYZ: %.1f %.1f %.1f", 
                    detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
            } else {
                telemetry.addLine("Pose data unavailable");
            }
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
