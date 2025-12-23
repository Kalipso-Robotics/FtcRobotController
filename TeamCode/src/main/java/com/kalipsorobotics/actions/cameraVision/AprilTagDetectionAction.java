package com.kalipsorobotics.actions.cameraVision;

import static com.kalipsorobotics.math.MathFunctions.angleWrapRad;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.math.LimelightPos;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class AprilTagDetectionAction extends Action {

    private boolean useWebcam = false;

//    private AprilTagProcessor aprilTagProcessor;
//    private VisionPortal visionPortal;
//    WebcamName camera;

    private final Limelight3A limelight;
    private final Turret turret;
    private final int targetAprilTagId;
    private boolean hasStarted = false;
    // Field-facing yaw of each goal tag (absolute field frame, +CCW from field X)
    private final double APRIL_YAW_FIELD_RAD = Math.toRadians(45); // todo measure final

    private final double APRILTAG_X_FROM_INIT_MM = (15 + 24*4 + 10) * 25.4; //todo measure final

    private final double APRILTAG_Y_FROM_INIT_MM = (7+32) * 25.4; //todo measure final

    private final double TURRET_CENTER_TO_LIMELIGHT_DIST_MM = 6.5*25.4; //todo measure final

    private final double ROBOT_CENTER_TO_TURRET_DISTANCE_MM = 4.0; //todo measure final

    double kDistanceScale = (72.0 + 59 + 32 + 25) / (72 + 58.5 + 31.25 + 23.5); // real / calculated

    boolean hasFound;
    private  double xCam;
    private double yCam;
    private double zCam;
    private double tagCamFlatDist;
    private double cameraHeadingRelAprilTag;


    public AprilTagDetectionAction(OpModeUtilities opModeUtilities, Turret turret, int targetAprilTagId) {
//        camera = opModeUtilities.getHardwareMap().get(WebcamName.class, "Webcam 1");
//        this.aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
//        visionPortal = VisionPortal.easyCreateWithDefaults(
//                camera,
//                aprilTagProcessor
//        );

        limelight = opModeUtilities.getHardwareMap().get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        this.turret = turret;
        this.targetAprilTagId = targetAprilTagId;
    }

    public void setUseWebcam(boolean useWebcam) {
        this.useWebcam = useWebcam;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    /**
     * Public method to trigger odometry update from vision
     * For use in testing and manual triggering
     */
    public void updateOdometryFromVision() {
        update();
    }

    @Override
    protected void update() {
        if (!hasStarted) {
            KLog.d("limelight_pos", "limelight started");
            hasStarted = true;
        }

        LLResult result = limelight.getLatestResult();
//        KLog.d("limelight", String.format("limelight results %s", result));
        hasFound = false;
        if (result != null && result.isValid()) {
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {

                int tagId = fr.getFiducialId();

//                boolean isRedGoal  = SharedData.getAllianceColor() == AllianceColor.RED  && tagId == 24;
//                boolean isBlueGoal = SharedData.getAllianceColor() == AllianceColor.BLUE && tagId == 20;
                KLog.d("limelight_pos", "tagId " + tagId);
                if (tagId == targetAprilTagId) {
                    KLog.d("limelight_pos", "tagId matches targetId " + targetAprilTagId);
                    hasFound = true;
                    Pose3D tagCamPose = fr.getTargetPoseCameraSpace();

                    xCam = tagCamPose.getPosition().x * 1000; // left right offset from tag
                    yCam = tagCamPose.getPosition().y * 1000;
                    zCam = tagCamPose.getPosition().z * 1000; // front back offset from tag

                    tagCamFlatDist = Math.hypot(xCam, zCam); //distance from apriltag center to camera center FLAT WITHOUT HEIGHT
                    KLog.d("limelight_pos", String.format("Distance to GOAL %.2f", tagCamFlatDist));
//                        double flatDist = Math.sqrt(tagCamFlatDist*tagCamFlatDist - deltaH*deltaH) * kDistanceScale;
//
                    cameraHeadingRelAprilTag = -(Math.atan2(xCam, zCam)); // angle of incidence from apriltag center to camera center
                    LimelightPos currentPos = new LimelightPos(tagCamFlatDist, cameraHeadingRelAprilTag, xCam, yCam, zCam);
                    SharedData.setLimelightPosition(currentPos);



                    KLog.d("limelight_pos", String.format("Limelight Pos: %s", currentPos));
                    KLog.d("limelight", String.format("Heading to tag (rad) %.3f", cameraHeadingRelAprilTag));
                    KLog.d("limelight_pos", String.format("LL GOAL TAG %d", tagId));
                    KLog.d("limelight_pos", String.format("Goal X (left-right mm) %.2f", xCam));
                    KLog.d("limelight_pos", String.format("Goal Z (front-back mm) %.2f", zCam));

                    // Calculate and set global robot position from vision
                    Position globalPos = calculateGlobalLimelightPosition();
                    if (globalPos != null) {
                        SharedData.setLimelightGlobalPosition(globalPos);
                    }
                }

                KLog.d("limelight_fiducial", String.format("Fiducial - ID: %d, Family: %s, Xdeg: %.2f, Ydeg: %.2f",
                        fr.getFiducialId(), fr.getFamily(),
                        fr.getTargetXDegrees(), fr.getTargetYDegrees()));
            }
        }

        if (!hasFound) {
            SharedData.getLimelightPosition().reset();
        }
    }

    /**
     * Transforms robot position from AprilTag-relative to field-origin-relative coordinates.
     * Uses the configured AprilTag position and orientation on the field.
     *
     * @param robotRelTagX Robot's X offset from AprilTag in AprilTag's local frame (mm)
     * @param robotRelTagY Robot's Y offset from AprilTag in AprilTag's local frame (mm)
     * @param robotRelTagTheta Robot's heading relative to AprilTag (radians)
     * @return Robot's position in global field coordinates
     */
    private Position transformToGlobalPosition(double robotRelTagX, double robotRelTagY, double robotRelTagTheta) {
        Position localPos = new Position(robotRelTagX, robotRelTagY, robotRelTagTheta);
        Position aprilTagFieldPos = new Position(APRILTAG_X_FROM_INIT_MM, APRILTAG_Y_FROM_INIT_MM, APRIL_YAW_FIELD_RAD);
        Position globalPos = localPos.toGlobal(aprilTagFieldPos);
        KLog.d("limelight_pos", String.format("AprilTag Field Pos: %s", aprilTagFieldPos));
        KLog.d("limelight_pos", String.format("Local Robot Pos: %s", localPos));
        KLog.d("limelight_pos", String.format("Global Robot Pos: %s",globalPos));
        return globalPos;
    }

    /**
     * Calculates the robot's global position based on current vision data.
     * Uses the detected AprilTag position and known tag field location.
     *
     * Coordinate system:
     * - Field: +X is forward from init position, +Y is left, angles are CCW from +X
     * - Camera: +Z is forward (optical axis), +X is right, +Y is down
     * - AprilTag's local frame: +X is the direction the tag faces, origin at tag center
     *
     * @return Robot's global field position, or null if no valid detection
     */
    public Position calculateGlobalLimelightPosition() {
        if (!hasFound) {
            KLog.d("limelight_pos", "No valid detection");
            return null;
        }


        // Camera position relative to AprilTag in tag's local frame:
        // Camera is in front of the tag (positive X in tag's frame)
        // Transform to field coordinates using transformToGlobalPosition
        Position camRelFieldPos = transformToGlobalPosition(tagCamFlatDist, 0, cameraHeadingRelAprilTag);
        KLog.d("limelight_pos", String.format("Turret Pos: %s", camRelFieldPos));

        // Robot heading = camera heading - turret angle
        double currentCamRelRobotRad = turret.getCurrentAngleRad();
        double robotHeadingField = angleWrapRad(camRelFieldPos.getTheta() - currentCamRelRobotRad);

        // Turret heading in field frame
        double camRelFieldRad = angleWrapRad(robotHeadingField + currentCamRelRobotRad);


        Position turretRelToField = camRelFieldPos.toGlobal(TURRET_CENTER_TO_LIMELIGHT_DIST_MM, TURRET_CENTER_TO_LIMELIGHT_DIST_MM, camRelFieldRad);
        KLog.d("limelight_pos", String.format("Turret Rel To Field: %s", turretRelToField));

        Position robotCenterRelToField = turretRelToField.toGlobal(ROBOT_CENTER_TO_TURRET_DISTANCE_MM, ROBOT_CENTER_TO_TURRET_DISTANCE_MM, camRelFieldRad);
        KLog.d("limelight_pos", String.format("Robot Center Rel To Field: %s", robotCenterRelToField));

        return robotCenterRelToField;
    }

}
