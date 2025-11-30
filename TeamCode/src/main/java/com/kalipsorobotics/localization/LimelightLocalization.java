package com.kalipsorobotics.localization;

import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Limelight Localization System
 *
 * Encapsulates Limelight3A vision sensor for robot localization using AprilTag fiducials.
 * Similar architecture to Odometry class - singleton pattern with simple public methods.
 *
 * Key Features:
 * - Detects AprilTag fiducials on the field
 * - Calculates robot position based on tag positions
 * - Corrects Odometry drift when updateOdometry() is called
 * - Low-power design: only runs when explicitly called (e.g., back button in TeleOp)
 *
 * Usage:
 *   LimelightLocalization limelight = LimelightLocalization.getInstance(opModeUtilities, odometry);
 *   if (gamepad1.back) {
 *       limelight.updateOdometry();  // Corrects odometry position from vision
 *   }
 *   double distance = limelight.getDistanceToTarget(targetTagId);
 */
public class LimelightLocalization {

    // Physical constants (in mm)
    private static final double CAMERA_HEIGHT_MM = 12.5 * 25.4;  // 12.5 inches off floor
    private static final double TAG_HEIGHT_MM = 29.5 * 25.4;     // 29.5 inches off floor
    private static final double DISTANCE_SCALE_FACTOR = (30.0 + 25) / (32 + 27);

    // Pipeline configuration
    private static final int DEFAULT_PIPELINE = 0;  // AprilTag detection pipeline

    // Singleton instance
    private static LimelightLocalization single_instance = null;

    // Hardware
    private Limelight3A limelight;
    private OpModeUtilities opModeUtilities;
    private Odometry odometry;

    // Cached result
    private LLResult latestResult = null;
    private boolean limelightStarted = false;

    /**
     * Private constructor for singleton pattern
     */
    private LimelightLocalization(OpModeUtilities opModeUtilities, Odometry odometry) {
        this.opModeUtilities = opModeUtilities;
        this.odometry = odometry;
        initializeHardware();
    }

    /**
     * Get singleton instance of LimelightLocalization
     * Creates new instance on first call, reuses on subsequent calls
     *
     * @param opModeUtilities Hardware access utilities
     * @param odometry Odometry instance to correct
     * @return LimelightLocalization singleton instance
     */
    public static synchronized LimelightLocalization getInstance(OpModeUtilities opModeUtilities, Odometry odometry) {
        if (single_instance == null) {
            single_instance = new LimelightLocalization(opModeUtilities, odometry);
            KLog.d("LimelightLocalization", "Created new instance");
        } else {
            single_instance.opModeUtilities = opModeUtilities;
            single_instance.odometry = odometry;
            KLog.d("LimelightLocalization", "Reusing existing instance");
        }
        return single_instance;
    }

    /**
     * Reset singleton instance (useful for testing or OpMode transitions)
     */
    public static void setInstanceNull() {
        if (single_instance != null && single_instance.limelightStarted) {
            single_instance.stop();
        }
        single_instance = null;
    }

    /**
     * Initialize Limelight hardware
     */
    private void initializeHardware() {
        try {
            limelight = opModeUtilities.getHardwareMap().get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(DEFAULT_PIPELINE);
            KLog.d("LimelightLocalization", "Hardware initialized successfully");
        } catch (Exception e) {
            KLog.e("LimelightLocalization", "Failed to initialize hardware: " + e.getMessage());
            throw new RuntimeException("Limelight hardware initialization failed", e);
        }
    }

    /**
     * Start the Limelight (begins capturing frames and processing)
     * Call this before attempting to get results
     */
    public void start() {
        if (!limelightStarted) {
            limelight.start();
            limelightStarted = true;
            KLog.d("LimelightLocalization", "Limelight started");
        }
    }

    /**
     * Stop the Limelight (saves power)
     * Call this when vision is no longer needed
     */
    public void stop() {
        if (limelightStarted) {
            limelight.stop();
            limelightStarted = false;
            KLog.d("LimelightLocalization", "Limelight stopped");
        }
    }

    /**
     * Update latest result from Limelight
     * Call this periodically to get fresh vision data
     *
     * @return true if valid result available, false otherwise
     */
    public boolean updateResult() {
        if (!limelightStarted) {
            start();
        }

        latestResult = limelight.getLatestResult();
        return latestResult != null && latestResult.isValid();
    }

    /**
     * Update Odometry position based on Limelight vision
     * This is the main method to call when you want to correct odometry drift
     *
     * Uses AprilTag fiducials to calculate robot's true position and updates:
     * - Odometry instance (if provided)
     * - SharedData odometry position
     *
     * @return true if position was successfully updated, false if no valid tags seen
     */
    public boolean updateOdometry() {
        if (!updateResult()) {
            KLog.d("LimelightLocalization", "No valid Limelight result - odometry not updated");
            return false;
        }

        List<LLResultTypes.FiducialResult> fiducialResults = latestResult.getFiducialResults();
        if (fiducialResults.isEmpty()) {
            KLog.d("LimelightLocalization", "No fiducials detected - odometry not updated");
            return false;
        }

        // Use the first detected tag to calculate position
        // TODO: In future, could average multiple tags or use tag with best confidence
        LLResultTypes.FiducialResult bestTag = fiducialResults.get(0);
        Position correctedPosition = calculatePositionFromTag(bestTag);

        if (correctedPosition == null) {
            KLog.d("LimelightLocalization", "Failed to calculate position from tag");
            return false;
        }

        // Update both Odometry and SharedData
        if (odometry != null) {
            // Update odometry's internal position
            // Note: Odometry doesn't have a public setPosition method, so we update SharedData
            // and rely on the odometry updating from SharedData or being reset with this position
            KLog.d("LimelightLocalization", "Odometry instance available, but no direct setter");
        }

        SharedData.setOdometryPosition(correctedPosition);
        KLog.d("LimelightLocalization", String.format(
            "Odometry updated from tag %d: X=%.1fmm, Y=%.1fmm, Theta=%.2frad",
            bestTag.getFiducialId(), correctedPosition.getX(), correctedPosition.getY(), correctedPosition.getTheta()));

        return true;
    }

    /**
     * Calculate robot position from a detected AprilTag
     *
     * @param fiducialResult The detected fiducial/AprilTag
     * @return Robot position, or null if calculation fails
     */
    private Position calculatePositionFromTag(LLResultTypes.FiducialResult fiducialResult) {
        try {
            // Get tag position relative to camera
            Pose3D tagCamPose = fiducialResult.getTargetPoseCameraSpace();

            // Convert to mm
            double xCam = tagCamPose.getPosition().x * 1000;
            double yCam = tagCamPose.getPosition().y * 1000;
            double zCam = tagCamPose.getPosition().z * 1000;

            // Calculate 3D distance to tag
            double goalDist3D = Math.sqrt(xCam*xCam + yCam*yCam + zCam*zCam);

            // Calculate flat distance on floor (accounting for height difference)
            double deltaH = TAG_HEIGHT_MM - CAMERA_HEIGHT_MM;
            double flatDist = Math.sqrt(goalDist3D*goalDist3D - deltaH*deltaH) * DISTANCE_SCALE_FACTOR;

            // Calculate heading to tag
            double headingToTagRad = Math.atan2(xCam, zCam);

            // TODO: Use tag's known field position to calculate robot's field position
            // For now, we return relative position (would need AprilTag field map)
            // This is a placeholder - you'll need to add tag positions for your field

            // Get tag's known field position
            Position tagFieldPosition = getTagFieldPosition(fiducialResult.getFiducialId());
            if (tagFieldPosition == null) {
                KLog.d("LimelightLocalization", "Unknown tag ID: " + fiducialResult.getFiducialId());
                return null;
            }

            // Calculate robot position based on tag position and camera observation
            // Robot is at: tag_position - (distance * direction_to_robot)
            double robotX = tagFieldPosition.getX() - flatDist * Math.sin(headingToTagRad + tagFieldPosition.getTheta());
            double robotY = tagFieldPosition.getY() - flatDist * Math.cos(headingToTagRad + tagFieldPosition.getTheta());
            double robotTheta = tagFieldPosition.getTheta() - headingToTagRad;

            return new Position(robotX, robotY, robotTheta);

        } catch (Exception e) {
            KLog.e("LimelightLocalization", "Error calculating position: " + e.getMessage());
            return null;
        }
    }

    /**
     * Get field position of an AprilTag by ID
     * TODO: Fill in with actual tag positions for your field
     *
     * @param tagId AprilTag ID
     * @return Tag's field position, or null if unknown
     */
    private Position getTagFieldPosition(int tagId) {
        // TODO: Add your field's AprilTag positions here
        // Example for FTC 2024-2025 INTO THE DEEP:
        switch (tagId) {
            // Red alliance tags
            case 11: return new Position(0, 0, 0);  // Red left
            case 12: return new Position(0, 1219, 0);  // Red center
            case 13: return new Position(0, 2438, 0);  // Red right

            // Blue alliance tags
            case 14: return new Position(3658, 2438, Math.PI);  // Blue right
            case 15: return new Position(3658, 1219, Math.PI);  // Blue center
            case 16: return new Position(3658, 0, Math.PI);  // Blue left

            default:
                KLog.d("LimelightLocalization", "Unknown tag ID: " + tagId);
                return null;
        }
    }

    /**
     * Get distance to a specific target tag
     *
     * @param targetTagId AprilTag ID to measure distance to
     * @return Distance in mm, or -1 if tag not visible
     */
    public double getDistanceToTarget(int targetTagId) {
        if (!updateResult()) {
            return -1;
        }

        List<LLResultTypes.FiducialResult> fiducialResults = latestResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            if (fr.getFiducialId() == targetTagId) {
                return calculateDistanceToTag(fr);
            }
        }

        KLog.d("LimelightLocalization", "Target tag " + targetTagId + " not visible");
        return -1;
    }

    /**
     * Get angle to a specific target tag
     *
     * @param targetTagId AprilTag ID to measure angle to
     * @return Angle in radians, or NaN if tag not visible
     */
    public double getAngleToTarget(int targetTagId) {
        if (!updateResult()) {
            return Double.NaN;
        }

        List<LLResultTypes.FiducialResult> fiducialResults = latestResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            if (fr.getFiducialId() == targetTagId) {
                Pose3D tagCamPose = fr.getTargetPoseCameraSpace();
                double xCam = tagCamPose.getPosition().x * 1000;
                double zCam = tagCamPose.getPosition().z * 1000;
                return Math.atan2(xCam, zCam);
            }
        }

        KLog.d("LimelightLocalization", "Target tag " + targetTagId + " not visible");
        return Double.NaN;
    }

    /**
     * Calculate distance to a detected tag
     */
    private double calculateDistanceToTag(LLResultTypes.FiducialResult fiducialResult) {
        Pose3D tagCamPose = fiducialResult.getTargetPoseCameraSpace();

        double xCam = tagCamPose.getPosition().x * 1000;
        double yCam = tagCamPose.getPosition().y * 1000;
        double zCam = tagCamPose.getPosition().z * 1000;

        double goalDist3D = Math.sqrt(xCam*xCam + yCam*yCam + zCam*zCam);
        double deltaH = TAG_HEIGHT_MM - CAMERA_HEIGHT_MM;

        return Math.sqrt(goalDist3D*goalDist3D - deltaH*deltaH) * DISTANCE_SCALE_FACTOR;
    }

    /**
     * Check if any AprilTags are currently visible
     *
     * @return true if at least one tag is visible
     */
    public boolean hasTarget() {
        if (!updateResult()) {
            return false;
        }

        List<LLResultTypes.FiducialResult> fiducialResults = latestResult.getFiducialResults();
        return !fiducialResults.isEmpty();
    }

    /**
     * Get list of all visible tag IDs
     *
     * @return Array of visible tag IDs, empty array if none visible
     */
    public int[] getVisibleTagIds() {
        if (!updateResult()) {
            return new int[0];
        }

        List<LLResultTypes.FiducialResult> fiducialResults = latestResult.getFiducialResults();
        int[] tagIds = new int[fiducialResults.size()];

        for (int i = 0; i < fiducialResults.size(); i++) {
            tagIds[i] = fiducialResults.get(i).getFiducialId();
        }

        return tagIds;
    }

    /**
     * Get the Limelight hardware object for advanced users
     *
     * @return Limelight3A hardware instance
     */
    public Limelight3A getLimelight() {
        return limelight;
    }

    /**
     * Get the latest result for advanced processing
     *
     * @return Latest LLResult, or null if no result available
     */
    public LLResult getLatestResult() {
        return latestResult;
    }

    /**
     * Switch to a different pipeline
     *
     * @param pipelineIndex Pipeline number (0-9)
     */
    public void setPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
        KLog.d("LimelightLocalization", "Switched to pipeline " + pipelineIndex);
    }
}
