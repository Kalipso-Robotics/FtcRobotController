package com.kalipsorobotics.actions.cameraVision;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class GoalDetectionAction extends Action {

    private boolean useWebcam = false;

//    private AprilTagProcessor aprilTagProcessor;
//    private VisionPortal visionPortal;
//    WebcamName camera;

    private Limelight3A limelight;
    private Turret turret;

    private final double APRILTAG_X_FROM_INIT = (15 + 24*4 + 10) * 25.4; //todo measure final

    private final double APRILTAG_Y_FROM_INIT = (7+32) * 25.4; //todo measure final

    private final double TURRET_CENTER_TO_LIMELIGHT_DIST = 6.5*25.4; //todo measure final

    private final double ROBOT_CENTER_TO_TURRET_DISTANCE = 4.0; //todo measure final

    final double CAM_HEIGHT_M = 12.5 * 25.4; //todo need to change for real bot
    final double TAG_HEIGHT_M = 29.5 * 25.4;
    final double deltaH = TAG_HEIGHT_M - CAM_HEIGHT_M;

    final double kDistanceScale = (30.0 + 25) / (32 + 27); //can add more data values

    public GoalDetectionAction(OpModeUtilities opModeUtilities, Turret turret) {
//        camera = opModeUtilities.getHardwareMap().get(WebcamName.class, "Webcam 1");
//        this.aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
//        visionPortal = VisionPortal.easyCreateWithDefaults(
//                camera,
//                aprilTagProcessor
//        );

        limelight = opModeUtilities.getHardwareMap().get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        this.turret = turret;
    }

    public void setUseWebcam(boolean useWebcam) {
        this.useWebcam = useWebcam;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    @Override
    protected void update() {

//        limelight.start();
//        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

//        if (useWebcam) {
//            for (AprilTagDetection detection : currentDetections) {
//                if (detection.metadata != null) {
//                    double distanceXMidGoal = detection.ftcPose.x + 25.4;
//                    double distanceYMidGoal = detection.ftcPose.y + 25.4;
//                    double distanceToGoal = Math.sqrt((distanceXMidGoal * distanceXMidGoal) + (distanceYMidGoal * distanceYMidGoal));
//                    if ((SharedData.getAllianceColor() == AllianceSetup.RED && detection.id == 24) || (SharedData.getAllianceColor() == AllianceSetup.BLUE && detection.id == 21)) {
//                        SharedData.setDistanceToGoal(distanceToGoal);
//                    }
//
//                    double angleTargetRadian = Math.atan2(distanceYMidGoal, distanceXMidGoal);
//                    SharedData.setAngleRadToGoal(angleTargetRadian);
//                } else {
//                    KLog.d("goaldetection", "camera cannot find apriltag");
//                }
//            }
//        } else {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {

                    int tagId = fr.getFiducialId();
                    boolean isRedGoal  = SharedData.getAllianceColor() == AllianceColor.RED  && tagId == 24;
                    boolean isBlueGoal = SharedData.getAllianceColor() == AllianceColor.BLUE && tagId == 21;

                    if (isRedGoal || isBlueGoal) {
                        Pose3D tagCamPose = fr.getTargetPoseCameraSpace();

                        double xCam = tagCamPose.getPosition().x * 1000;
                        double yCam = tagCamPose.getPosition().y * 1000;
                        double zCam = tagCamPose.getPosition().z * 1000;

                        double goalDist3D = Math.sqrt(xCam*xCam + yCam*yCam + zCam*zCam);

                        double flatDist = Math.sqrt(goalDist3D*goalDist3D - deltaH*deltaH) * kDistanceScale;

                        double headingRad = (Math.atan2(xCam, zCam));
                        telemetry.addData("Heading to tag (rad)", "%.3f", headingRad);

                        double currentTurretRad = turret.getCurrentAngleRad();
                        double currentRobotRad = SharedData.getOdometryPosition().getTheta();
                        double totalAngleToTag = currentRobotRad + currentTurretRad + headingRad;

                        double xDistCamToTag = Math.cos(totalAngleToTag) * flatDist;
                        double yDistCamToTag = Math.sin(totalAngleToTag) * flatDist;

                        double xDistTurretToLimelight = Math.cos(currentTurretRad) * TURRET_CENTER_TO_LIMELIGHT_DIST;
                        double yDistTurretToLimeLight = Math.sin(currentTurretRad) * TURRET_CENTER_TO_LIMELIGHT_DIST;

                        double xDistCenterToTurret = Math.cos(currentRobotRad) * ROBOT_CENTER_TO_TURRET_DISTANCE;
                        double yDistCenterToTurret = Math.sin(currentRobotRad) * ROBOT_CENTER_TO_TURRET_DISTANCE;

                        double totalX = xDistCamToTag + xDistTurretToLimelight + xDistCenterToTurret;
                        double totalY = yDistCamToTag + yDistTurretToLimeLight + yDistCenterToTurret;

                        SharedData.setOdometryPosition(new Position(APRILTAG_X_FROM_INIT - totalX, APRILTAG_Y_FROM_INIT - totalY, SharedData.getOdometryPosition().getTheta()));

                        // 2D distance ignoring height
//                        double distance = Math.hypot(totalX, totalY);

//                        SharedData.setDistanceToGoal(distance);
//                        SharedData.setAngleRadToGoal(angleRad);

                        telemetry.addData("LL GOAL TAG", tagId);
                        telemetry.addData("Goal X (mm)", "%.3f", xCam);
                        telemetry.addData("Goal Z (mm)", "%.3f", zCam);
                    }

                    // existing debug
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, Xdeg: %.2f, Ydeg: %.2f",
                            fr.getFiducialId(), fr.getFamily(),
                            fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

            } else {
                telemetry.addLine("Results invalid.");
            }

            telemetry.update();

//        }

//        limelight.stop();
    }

}
