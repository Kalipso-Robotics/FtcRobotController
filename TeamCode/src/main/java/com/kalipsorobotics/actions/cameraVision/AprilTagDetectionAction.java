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

    private final OpModeUtilities opModeUtilities;

    private final Limelight3A limelight;
    private final Turret turret;
    private final int targetAprilTagId;
    private boolean hasStarted = false;
    private final Position aprilTagRelFieldPos;

    private Position camRelAprilTagPos;

    boolean hasFound;
    private double xAprilTagRelToCamMM;
    private double yAprilTagRelToCamMM;
    private double zAprilTagRelToCamMM;
    private double distanceFromCamToAprilTag;

    private double prevPitchDeg = Double.MIN_VALUE;


    public AprilTagDetectionAction(OpModeUtilities opModeUtilities, Turret turret, int targetAprilTagId, AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
        this.opModeUtilities = opModeUtilities;

        aprilTagRelFieldPos =  new Position(APRILTAG_X_REL_FIELD_MM, APRILTAG_Y_REL_FIELD_MM * allianceColor.getPolarity(), APRIL_TAG_HEADING_REL_FIELD_RAD * allianceColor.getPolarity());

        limelight = opModeUtilities.getHardwareMap().get(Limelight3A.class, "limelight");

        if (allianceColor == AllianceColor.RED) {
            boolean pipelineSwitched = limelightSwitchPipeline(0);
            KLog.d("AprilTagDetection_Pipeline", "Pipeline Switch Red " + pipelineSwitched);
        } else {
            boolean pipelineSwitched = limelightSwitchPipeline(1);
            KLog.d("AprilTagDetection_Pipeline", "Pipeline Switch Blue " + pipelineSwitched);
        }

        limelight.start();


        this.turret = turret;
        this.targetAprilTagId = targetAprilTagId;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    @Override
    protected void update() {
        if (!hasStarted) {
            KLog.d("AprilTagDetection_Start", "limelight started");
            hasStarted = true;
        }

        LLResult result = limelight.getLatestResult();
        KLog.d("AprilTagDetection_Result", "Result validity, " + result.isValid() + " result: " + result);
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
                    double camRelAprilTagPitchDeg = camRelAprilTagPose.getOrientation().getPitch();
                    boolean isSpike = isLimelightSpike(camRelAprilTagPitchDeg, prevPitchDeg);
                    prevPitchDeg = camRelAprilTagPitchDeg;
                    if (isSpike) {
                        SharedData.getLimelightRawPosition().reset();
                        hasFound = false;
                        return;
                    }
                    // Counter Clockwise is negative and clockwise is positive
                    double camRelAprilTagTheta = MathFunctions.angleWrapRad(Math.toRadians(90 + camRelAprilTagPitchDeg));
                    camRelAprilTagPos = new Position(-camRelAprilTagPose.getPosition().z * 1000, -camRelAprilTagPose.getPosition().x * 1000, camRelAprilTagTheta);
                    KLog.d("AprilTagDetection_limelight_pos", "camRelAprilTag Position (after transform): " + camRelAprilTagPos);


                    // Calculate and set global robot position from vision
                    Position globalPos = calculateGlobalLimelightPosition();
                    if (globalPos != null) {

                        KLog.d("AprilTagDetection_limelight_pos_global", "Updating LimeLight share data robot position to: " + globalPos);
                        SharedData.setLimelightGlobalPosition(globalPos);
                    }

                    //========================= For Raw Data Stuff To April Tag ======================
                    xAprilTagRelToCamMM = aprilTagRelCamPose.getPosition().x * 1000 * allianceColor.getPolarity();
                    yAprilTagRelToCamMM = aprilTagRelCamPose.getPosition().y * 1000;
                    zAprilTagRelToCamMM = aprilTagRelCamPose.getPosition().z * 1000; // front back offset from tag

                    // ANGLE -----------
                    double estimateHeadingFromCamToGoal = Math.atan2(xAprilTagRelToCamMM + (Math.signum(xAprilTagRelToCamMM)) * GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_X, zAprilTagRelToCamMM + (GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_Z / 2));

                    // DISTANCE -----------
                    distanceFromCamToAprilTag = Math.hypot(xAprilTagRelToCamMM, zAprilTagRelToCamMM);

                    // SEND TO SHARED DATA ----------
                    LimelightPos currentRawPos = new LimelightPos(distanceFromCamToAprilTag, estimateHeadingFromCamToGoal, xAprilTagRelToCamMM, yAprilTagRelToCamMM, zAprilTagRelToCamMM);
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

    private boolean limelightSwitchPipeline(int index) {
        boolean limelightSwitched = limelight.pipelineSwitch(index);
        if (!limelightSwitched) {
            opModeUtilities.getOpMode().sleep(150);
            limelightSwitched = limelight.pipelineSwitch(index);
        }
        return limelightSwitched;
    }

    private boolean isLimelightSpike(double currentPitchDeg, double prevPitchDeg) {
        double angleDiff = MathFunctions.angleWrapDeg(currentPitchDeg - prevPitchDeg);
        boolean isSpike = (Math.abs(angleDiff) > (11));
        if (isSpike) {
            KLog.d("AprilTagDetection_limelight_pos", "Pitch spike. Skip update and reset. from: " + prevPitchDeg + " to:" + currentPitchDeg);
        }
        return isSpike;
    }


}
