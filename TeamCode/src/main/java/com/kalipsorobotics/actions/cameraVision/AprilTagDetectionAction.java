package com.kalipsorobotics.actions.cameraVision;

import static com.kalipsorobotics.math.MathFunctions.angleWrapRad;

import com.kalipsorobotics.actions.actionUtilities.Action;
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

    private final double APRILTAG_X_FROM_INIT = (15 + 24*4 + 10) * 25.4; //todo measure final

    private final double APRILTAG_Y_FROM_INIT = (7+32) * 25.4; //todo measure final

    private final double TURRET_CENTER_TO_LIMELIGHT_DIST = 6.5*25.4; //todo measure final

    private final double ROBOT_CENTER_TO_TURRET_DISTANCE = 4.0; //todo measure final

    final double CAM_HEIGHT_M = 12.5 * 25.4; //todo need to change for real bot
    final double TAG_HEIGHT_M = 29.5 * 25.4;
    final double deltaH = TAG_HEIGHT_M - CAM_HEIGHT_M;

    double kDistanceScale = (72.0 + 59 + 32 + 25) / (72 + 58.5 + 31.25 + 23.5); // real / calculated

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
        boolean hasFound = false;
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

                    double xCam = tagCamPose.getPosition().x * 1000; // left right offset from tag
                    double yCam = tagCamPose.getPosition().y * 1000;
                    double zCam = tagCamPose.getPosition().z * 1000; // front back offset from tag

                    double tagCamFlatDist = Math.hypot(xCam, zCam); //distance from apriltag center to camera center FLAT WITHOUT HEIGHT
                    KLog.d("limelight_pos", String.format("Distance to GOAL %.2f", tagCamFlatDist));
//                        double flatDist = Math.sqrt(tagCamFlatDist*tagCamFlatDist - deltaH*deltaH) * kDistanceScale;
//
                    double headingRad = -(Math.atan2(xCam, zCam)); // angle of incidence from apriltag center to camera center
                    LimelightPos currentPos = new LimelightPos(tagCamFlatDist, headingRad, xCam, yCam, zCam);
                    SharedData.setLimelightPosition(currentPos);

                    KLog.d("limelight_pos", String.format("Limelight Pos: %s", currentPos));


                            // --------------------- odometry ----------------------------

                    KLog.d("limelight", String.format("Heading to tag (rad) %.3f", headingRad));

                    double camHeadingField = angleWrapRad(APRIL_YAW_FIELD_RAD + Math.PI - headingRad); // heading of limelight in relation to field

                    double currentTurretRad = turret.getCurrentAngleRad();

                    //------------- current robot rad -----------------
//                        double currentRobotRad = SharedData.getOdometryPosition().getTheta();

                    double currentRobotRad  = angleWrapRad(camHeadingField - currentTurretRad);

                    //------------- ================= -----------------

                    double bearingField = angleWrapRad(camHeadingField + headingRad);

                    double xDistCamToTag = Math.cos(bearingField) * tagCamFlatDist;
                    double yDistCamToTag = Math.sin(bearingField) * tagCamFlatDist;

                    double xDistTurretToLimelight = Math.cos(currentTurretRad + currentRobotRad) * TURRET_CENTER_TO_LIMELIGHT_DIST;
                    double yDistTurretToLimeLight = Math.sin(currentTurretRad + currentRobotRad) * TURRET_CENTER_TO_LIMELIGHT_DIST;

                    double xDistCenterToTurret = Math.cos(currentRobotRad) * ROBOT_CENTER_TO_TURRET_DISTANCE;
                    double yDistCenterToTurret = Math.sin(currentRobotRad) * ROBOT_CENTER_TO_TURRET_DISTANCE;

                    double totalX = xDistCamToTag + xDistTurretToLimelight + xDistCenterToTurret;
                    double totalY = yDistCamToTag + yDistTurretToLimeLight + yDistCenterToTurret;

//                    SharedData.setLimelightOdometryPosition(new Position(APRILTAG_X_FROM_INIT - totalX, APRILTAG_Y_FROM_INIT - totalY, currentRobotRad));

                    // 2D distance ignoring height
//                        double distance = Math.hypot(totalX, totalY);

//                        SharedData.setDistanceToGoal(distance);
//                        SharedData.setAngleRadToGoal(angleRad);

                    KLog.d("limelight_pos", String.format("LL GOAL TAG %d", tagId));
                    KLog.d("limelight_pos", String.format("Goal X (left-right mm) %.2f", xCam));
                    KLog.d("limelight_pos", String.format("Goal Z (front-back mm) %.2f", zCam));

                }

                // existing debug
                KLog.d("limelight_fiducial", String.format("Fiducial - ID: %d, Family: %s, Xdeg: %.2f, Ydeg: %.2f",
                        fr.getFiducialId(), fr.getFamily(),
                        fr.getTargetXDegrees(), fr.getTargetYDegrees()));
            }
        }

        if (!hasFound) {
            SharedData.getLimelightPosition().reset();
        }
    }

}
