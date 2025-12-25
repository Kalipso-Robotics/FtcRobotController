package com.kalipsorobotics.actions.cameraVision;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.math.Point;
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
    // Field-facing yaw of each goal tag (absolute field frame, +CCW from field X)

    //Perpendicular vector to AprilTag
    private final double APRIL_TAG_HEADING_REL_FIELD_RAD = -Math.toRadians(126); //Math.toRadians(-234);

    private final double APRILTAG_X_REL_FIELD_MM = 3305;

    private final double APRILTAG_Y_REL_FIELD_MM = 1016;

    private final double APRIL_X_REL_TO_GOAL = 300.232;

    private final double APRIL_Y_REL_TO_GOAL = 369.541;

    private final double APRIL_TAG_ANGLE_REL_TO_GOAL_RAD = Math.toRadians(54);

    private final Position aprilTagRelFieldPos;



    //Measure from CAD
//    private final double FIELD_ORIGIN_X_REL_APRILTAG_MM = 2629;
//
//    private final double FIELD_ORIGIN_Y_REL_APRILTAG_MM = -1894;

    private final Position TURRET_REL_CAM_POS = new Position(-169.8848, -3.2466, 0);

    private final Point ROBOT_REL_TURRET_POINT = new Point(3.48052, 2.50233);


    private Position camRelAprilTag;
    double kDistanceScale = (72.0 + 59 + 32 + 25) / (72 + 58.5 + 31.25 + 23.5); // real / calculated

    boolean hasFound;
    private double xCamMM;
    private double yCam;
    private double zCamMM;
    private double distanceFromCamToGoal;
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


//========================= For Distance Stuff ======================
                    xCamMM = camRelAprilTagPose.getPosition().x * 1000; // left right offset from tag
                    yCam = camRelAprilTagPose.getPosition().y * 1000;
                    zCamMM = camRelAprilTagPose.getPosition().z * 1000; // front back offset from tag
                    double pitchCamRad = Math.toRadians(camRelAprilTagPose.getOrientation().getPitch());

                    cameraPositionRelToGoal = calculateCamPositionInGoalSpace(xCamMM, zCamMM, pitchCamRad); // angle of incidence from apriltag center to camera center
                    double targetHeadingToGoal = Math.atan2(cameraPositionRelToGoal.getY(), cameraPositionRelToGoal.getX());
                    KLog.d("AprilTagDetection", "desired global target heading to goal " + targetHeadingToGoal + " rad, " + Math.toDegrees(targetHeadingToGoal) + " deg");


                    double headingToGoal = targetHeadingToGoal - cameraPositionRelToGoal.getTheta();
                    KLog.d("AprilTagDetection", "heading to goal " + headingToGoal + " rad, " + Math.toDegrees(headingToGoal) + " deg");


                    distanceFromCamToGoal = Math.hypot(cameraPositionRelToGoal.getX(), cameraPositionRelToGoal.getY()); //distance from apriltag center to camera center FLAT WITHOUT HEIGHT
                    KLog.d("AprilTagDetection_limelight_pos", "Distance to GOAL " + distanceFromCamToGoal);

                    // GOAL IS NOT APRIL TAG ANYMORE BUT IS THE CORNER OF THE GOAL

                    LimelightPos currentPos = new LimelightPos(distanceFromCamToGoal, headingToGoal, xCamMM, yCam, zCamMM);
                    KLog.d("AprilTagDetection_limelight_pos", "Set to SharedData. currentPos: " + currentPos);
                    SharedData.setLimelightPosition(currentPos);
//====================================== Odometry =====================================

                    //Limelight has wabi-sabi Limelight thinks yaw is pitch and x is z and y is x
                    //As you rotate whichever responds most to rotation is correct value.
                    camRelAprilTag = new Position(-camRelAprilTagPose.getPosition().z * 1000, -camRelAprilTagPose.getPosition().x * 1000, Math.toRadians(180 + camRelAprilTagPose.getOrientation().getPitch()));
                    KLog.d("AprilTagDetection_limelight_pos", "camRelAprilTag Position (after transform): " + camRelAprilTag);


                    // Calculate and set global robot position from vision
                    Position globalPos = calculateGlobalLimelightPosition();
                    if (globalPos != null) {
                        KLog.d("AprilTagDetection_limelight_pos_global", "Updating LimeLight share data robot position to: " + globalPos);
                        SharedData.setLimelightGlobalPosition(globalPos);
                    }
                }
            }
        }

        if (!hasFound) {
            SharedData.getLimelightPosition().reset();
        }
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

        Position robotRelRobot = new Position(0 ,0 ,0);

        double turretAngle = turret.getCurrentAngleRad();  // get current turret heading in radians

        Position robotRelTurret = robotRelRobot.toNewFrame(new Position(ROBOT_REL_TURRET_POINT.getX(), ROBOT_REL_TURRET_POINT.getY(), -turretAngle));
        KLog.d("AprilTagDetection_calculateGlobal", "robotRelTurret " + robotRelTurret);

        Position robotRelCam = robotRelTurret.toNewFrame(TURRET_REL_CAM_POS);
        KLog.d("AprilTagDetection_calculateGlobal", "robotRelCam " + robotRelCam);

        Position robotRelAprilTag = robotRelCam.toNewFrame(camRelAprilTag);
        KLog.d("AprilTagDetection_calculateGlobal", "robotRelAprilTag " + robotRelAprilTag);

        Position robotRelField = robotRelAprilTag.toNewFrame(aprilTagRelFieldPos);
        KLog.d("AprilTagDetection_calculateGlobal", "robotRelField " + robotRelField);

        return robotRelField;
    }

    public Position calculateCamPositionInGoalSpace(double camXInAprilTagSpace, double camZInAprilTagSpace, double camHeadingInAprilTagSpaceRad) {
        Position turretInAprilTagSpace = new Position(-camZInAprilTagSpace, -camXInAprilTagSpace, Math.toRadians(180) + camHeadingInAprilTagSpaceRad);
        KLog.d("AprilTagDetection", "turret in april tag space " + turretInAprilTagSpace.toString());
        Position turretInGoalSpace = turretInAprilTagSpace.toNewFrame(new Position(APRIL_X_REL_TO_GOAL, APRIL_Y_REL_TO_GOAL, APRIL_TAG_ANGLE_REL_TO_GOAL_RAD)); // from cad
        KLog.d("AprilTagDetection", "turet in goal space " + turretInGoalSpace.toString());
        return turretInGoalSpace;
    }
}
