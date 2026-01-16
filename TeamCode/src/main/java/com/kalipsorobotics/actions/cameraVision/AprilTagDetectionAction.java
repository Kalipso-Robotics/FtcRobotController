package com.kalipsorobotics.actions.cameraVision;

import static com.kalipsorobotics.decode.configs.AprilTagConfig.*;

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
    private int consecutiveGoodReadings = 0;
    private int consecutiveBadReadings;

    private static final int STABILITY_THRESHOLD = 2;
    public static int CONSECUTIVE_BAD_READING_TOLERANCE = 0;



    public AprilTagDetectionAction(OpModeUtilities opModeUtilities, Turret turret, int targetAprilTagId, AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
        this.opModeUtilities = opModeUtilities;

        aprilTagRelFieldPos =  new Position(APRILTAG_X_REL_FIELD_MM, APRILTAG_Y_REL_FIELD_MM * allianceColor.getPolarity(), APRIL_TAG_HEADING_REL_FIELD_RAD * allianceColor.getPolarity());

        limelight = opModeUtilities.getHardwareMap().get(Limelight3A.class, "limelight");

        if (allianceColor == AllianceColor.RED) {
            limelightSwitchPipeline(0);
        } else {
            limelightSwitchPipeline(1);
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
            hasStarted = true;
        }

        LLResult result = limelight.getLatestResult();
        hasFound = false;

        KLog.d("AprilTag", "AprilTagDetection is running. Is Result valid: " + result.isValid()+ " Full result:" + result);
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducialResult : fiducialResults) {
                int tagId = fiducialResult.getFiducialId();
                if (tagId == targetAprilTagId) {
                    hasFound = true;
                    Pose3D aprilTagRelCamPose = fiducialResult.getTargetPoseCameraSpace();
                    Pose3D camRelAprilTagPose = fiducialResult.getCameraPoseTargetSpace();

                    // ==================== RAW LIMELIGHT DATA ====================
                    double rawPitchDeg = camRelAprilTagPose.getOrientation().getPitch();
                    double rawCamPoseX = camRelAprilTagPose.getPosition().x;
                    double rawCamPoseZ = camRelAprilTagPose.getPosition().z;

                    // ==================== SPIKE DETECTION ====================
                    boolean isSpike = isLimelightSpike(rawPitchDeg, prevPitchDeg);
                    if (isSpike) {
                        KLog.d("AprilTag_SPIKE", String.format("REJECTED | prev=%.2f° curr=%.2f° delta=%.2f°",
                                prevPitchDeg, rawPitchDeg, rawPitchDeg - prevPitchDeg));
                        SharedData.getLimelightRawPosition().reset();
                        hasFound = false;
                        consecutiveGoodReadings = 0;
                        prevPitchDeg = rawPitchDeg;
                        return;
                    }
                    prevPitchDeg = rawPitchDeg;
                    consecutiveGoodReadings++;
                    consecutiveBadReadings = 0;
                    consecutiveGoodReadings = Math.min(consecutiveGoodReadings, STABILITY_THRESHOLD + 1);

                    // ==================== CALCULATED: Odometry Transform ====================
                    double camRelAprilTagTheta = MathFunctions.angleWrapRad(Math.toRadians(180 + rawPitchDeg));
                    camRelAprilTagPos = new Position(-rawCamPoseZ * 1000, -rawCamPoseX * 1000, camRelAprilTagTheta);

                    KLog.d("AprilTag_CAM_REL_APRIL_TAG", String.format("CamRelTag(x=%.1fmm, y=%.1fmm, θ=%.2f°)",
                            camRelAprilTagPos.getX(), camRelAprilTagPos.getY(), Math.toDegrees(camRelAprilTagTheta)));

                    // ==================== CALCULATED: Global Position ====================
                    Position globalPos = calculateGlobalLimelightPosition();
                    if (globalPos != null) {
                        SharedData.setLimelightGlobalPosition(globalPos);
                        KLog.d("AprilTag_GLOBAL", String.format("RobotPos(x=%.1fmm, y=%.1fmm, θ=%.2f°)",
                                globalPos.getX(), globalPos.getY(), Math.toDegrees(globalPos.getTheta())));
                    }
                    //========================= For Raw Data Stuff To April Tag ======================
                    xAprilTagRelToCamMM = aprilTagRelCamPose.getPosition().x * 1000;
                    yAprilTagRelToCamMM = aprilTagRelCamPose.getPosition().y * 1000;
                    zAprilTagRelToCamMM = aprilTagRelCamPose.getPosition().z * 1000; // front back offset from tag

                    // ==================== CALCULATED: Angle & Distance to Goal ====================
                    // Goal offset is fixed relative to AprilTag - always add in positive X direction
                    distanceFromCamToAprilTag = Math.hypot(xAprilTagRelToCamMM, zAprilTagRelToCamMM);
                    double adjustedX = xAprilTagRelToCamMM;
                    double adjustedZ = zAprilTagRelToCamMM + GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_Z;

                    double estimateHeadingFromCamToGoal = Math.atan2(adjustedX, adjustedZ);

                    // FOR RPS TUNING ---------------------
                    opModeUtilities.getTelemetry().addLine(String.format("Distance to April Tag: %.2f", distanceFromCamToAprilTag));
//                    opModeUtilities.getTelemetry().update();
                    // FOR RPS TUNING ---------------------

                    KLog.d("AprilTag_GOAL", String.format("TagPos(x=%.1f, z=%.1f) + Offset -> Adj(x=%.1f, z=%.1f) | AngleToGoal=%.2f° | Dist=%.1fmm",
                            xAprilTagRelToCamMM, zAprilTagRelToCamMM, adjustedX, adjustedZ,
                            Math.toDegrees(estimateHeadingFromCamToGoal), distanceFromCamToAprilTag));

                    // ==================== OUTPUT: Send to SharedData ====================
                    if (consecutiveGoodReadings > STABILITY_THRESHOLD) {
                        LimelightPos currentRawPos = new LimelightPos(distanceFromCamToAprilTag, estimateHeadingFromCamToGoal, xAprilTagRelToCamMM, yAprilTagRelToCamMM, zAprilTagRelToCamMM);
                        SharedData.setLimelightRawPosition(currentRawPos);
                    } else {
                        KLog.d("AprilTag_STABILITY", String.format("Waiting for stable readings: %d/%d", consecutiveGoodReadings, STABILITY_THRESHOLD));
                    }
                }
            }
        }

        if (!hasFound) {
            consecutiveGoodReadings = 0;
            consecutiveBadReadings++;
            if (consecutiveBadReadings > CONSECUTIVE_BAD_READING_TOLERANCE) {
                KLog.d("AprilTag", "No AprilTag detected consecutively. Resetting SharedData.");
                SharedData.getLimelightRawPosition().reset();
            }
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
            return null;
        }

        Position robotRelRobotPos = new Position(0, 0, 0);
        double turretAngle = turret.getCurrentAngleRad();

        Position robotRelTurretPos = robotRelRobotPos.toNewFrame(new Position(ROBOT_REL_TURRET_POINT.getX(), ROBOT_REL_TURRET_POINT.getY(), -turretAngle));
        Position robotRelCamPos = robotRelTurretPos.toNewFrame(TURRET_REL_CAM_POS);
        Position robotRelAprilTagPos = robotRelCamPos.toNewFrame(camRelAprilTagPos);
        Position robotRelFieldPos = robotRelAprilTagPos.toNewFrame(aprilTagRelFieldPos);
        KLog.d("AprilTag_GLOBAL", "robotRelFieldPos: " + robotRelFieldPos);
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
        return Math.abs(angleDiff) > 90;
    }


}
