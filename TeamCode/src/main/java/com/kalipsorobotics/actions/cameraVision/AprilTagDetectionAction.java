package com.kalipsorobotics.actions.cameraVision;

import static com.kalipsorobotics.actions.cameraVision.AprilTagConfig.*;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.math.MathFunctions;
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

    AllianceColor allianceColor;

    private boolean useWebcam = false;

//    private AprilTagProcessor aprilTagProcessor;
//    private VisionPortal visionPortal;
//    WebcamName camera;

    private final Limelight3A limelight;
    private final Turret turret;
    private final int targetAprilTagId;
    private boolean hasStarted = false;
    private final Position aprilTagRelFieldPos;

    private Position camRelAprilTagPos;

    boolean hasFound;
    private double xCamMM;
    private double yCamMM;
    private double zCamMM;
    private double distanceFromCamToAprilTag;
    private Position cameraPositionRelToGoal;


    public AprilTagDetectionAction(OpModeUtilities opModeUtilities, Turret turret, int targetAprilTagId, AllianceColor allianceColor) {
        this.allianceColor = allianceColor;

        aprilTagRelFieldPos =  new Position(APRILTAG_X_REL_FIELD_MM, APRILTAG_Y_REL_FIELD_MM * allianceColor.getPolarity(), APRIL_TAG_HEADING_REL_FIELD_RAD * allianceColor.getPolarity());

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
            KLog.d("AprilTagDetection_Start", "limelight started");
            hasStarted = true;
        }

        LLResult result = limelight.getLatestResult();
//        KLog.d("limelight", String.format("limelight results %s", result));
        hasFound = false;
        if (result != null && result.isValid()) {
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducialResult : fiducialResults) {

                int tagId = fiducialResult.getFiducialId();

                KLog.d("AprilTagDetection_Tag", "tagId " + tagId);
                if (tagId == targetAprilTagId) {
                    KLog.d("AprilTagDetection_limelight_pos", "tagId matches targetId " + targetAprilTagId);
                    hasFound = true;
                    Pose3D aprilTagRelCamPose = fiducialResult.getTargetPoseCameraSpace();
//                    KLog.d("AprilTagDetection_limelight_pos", "aprilTagRelCamPose: (x, y, z), angle: " + aprilTagRelCamPose);

                    //Pitch Counter Clockwise is negative and clockwise is positive
                    Pose3D camRelAprilTagPose = fiducialResult.getCameraPoseTargetSpace();
                    KLog.d("AprilTagDetection_limelight_pos", "camRelAprilTagPose: (x, y, z), angle: " + camRelAprilTagPose.toString());



//====================================== Odometry =====================================

                    //Limelight has wabi-sabi Limelight thinks yaw is pitch and x is z and y is x
                    //As you rotate whichever responds most to rotation is correct value.
                    double camRelAprilTagTheta = MathFunctions.angleWrapRad(Math.toRadians(180 + camRelAprilTagPose.getOrientation().getPitch()));
                    camRelAprilTagPos = new Position(-camRelAprilTagPose.getPosition().z * 1000, -camRelAprilTagPose.getPosition().x * 1000, camRelAprilTagTheta);
                    KLog.d("AprilTagDetection_limelight_pos", "camRelAprilTag Position (after transform): " + camRelAprilTagPos);


                    // Calculate and set global robot position from vision
                    Position globalPos = calculateGlobalLimelightPosition();
                    if (globalPos != null) {

                        KLog.d("AprilTagDetection_limelight_pos_global", "Updating LimeLight share data robot position to: " + globalPos);
                        SharedData.setLimelightGlobalPosition(globalPos);
                    }

                    //========================= For Raw Data Stuff To April Tag ======================
                    xCamMM = camRelAprilTagPose.getPosition().x * 1000; // left right offset from tag
                    yCamMM = camRelAprilTagPose.getPosition().y * 1000;
                    zCamMM = camRelAprilTagPose.getPosition().z * 1000; // front back offset from tag

                    double headingFromCamToAprilTag = Math.atan2(xCamMM, -zCamMM);
                    distanceFromCamToAprilTag = Math.hypot(xCamMM, -zCamMM);

                    LimelightPos currentRawPos = new LimelightPos(distanceFromCamToAprilTag, headingFromCamToAprilTag, xCamMM, yCamMM, zCamMM);
                    KLog.d("AprilTagDetection_limelight_pos", "Set to SharedData. currentRawPos: " + currentRawPos);
                    SharedData.setLimelightRawPosition(currentRawPos);
                }
            }
        }

        if (!hasFound) {
            SharedData.getLimelightRawPosition().reset();
        }
    }


    /**
     * Calculates the robot's global position based on current vision data.
     * Uses the detected AprilTag position and known tag field location.
     *
     * Coordinate system:
     * - Field: +X is forward from init position, +Y is right, angles are CCW from +X
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

        Position robotRelRobotPos = new Position(0 ,0 ,0);

        double turretAngle = turret.getCurrentAngleRad();  // get current turret heading in radians

        Position robotRelTurretPos = robotRelRobotPos.toNewFrame(new Position(ROBOT_REL_TURRET_POINT.getX(), ROBOT_REL_TURRET_POINT.getY(), -turretAngle));
        KLog.d("AprilTagDetection_calculateGlobal", "robotRelTurretPos " + robotRelTurretPos);

        Position robotRelCamPos = robotRelTurretPos.toNewFrame(TURRET_REL_CAM_POS);
        KLog.d("AprilTagDetection_calculateGlobal", "robotRelCamPos " + robotRelCamPos);

        Position robotRelAprilTagPos = robotRelCamPos.toNewFrame(camRelAprilTagPos);
        KLog.d("AprilTagDetection_calculateGlobal", "robotRelAprilTagPos " + robotRelAprilTagPos);

        Position robotRelFieldPos = robotRelAprilTagPos.toNewFrame(aprilTagRelFieldPos);
        KLog.d("AprilTagDetection_calculateGlobal", "robotRelFieldPos " + robotRelFieldPos);

        return robotRelFieldPos;
    }

    public Position calculateCamPositionInGoalSpace(double camXInAprilTagSpace, double camZInAprilTagSpace, double camHeadingInAprilTagSpaceRad) {
        Position turretInAprilTagSpace = new Position(-camZInAprilTagSpace, -camXInAprilTagSpace, Math.toRadians(180) + camHeadingInAprilTagSpaceRad);
        KLog.d("AprilTagDetection", "turret in april tag space " + turretInAprilTagSpace.toString());
        Position turretInGoalSpace = turretInAprilTagSpace.toNewFrame(new Position(APRIL_X_REL_TO_GOAL, APRIL_Y_REL_TO_GOAL, APRIL_TAG_ANGLE_REL_TO_GOAL_RAD)); // from cad
        KLog.d("AprilTagDetection", "turet in goal space " + turretInGoalSpace.toString());
        return turretInGoalSpace;
    }
}
