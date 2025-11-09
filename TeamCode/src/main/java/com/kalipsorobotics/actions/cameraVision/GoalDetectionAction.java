package com.kalipsorobotics.actions.cameraVision;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.cameraVision.AllianceSetup;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class GoalDetectionAction extends Action {

    private boolean useWebcam = false;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    WebcamName camera;

    private Limelight3A limelight;

    public GoalDetectionAction(OpModeUtilities opModeUtilities) {
        camera = opModeUtilities.getHardwareMap().get(WebcamName.class, "Webcam 1");
        this.aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                camera,
                aprilTagProcessor
        );

        limelight = opModeUtilities.getHardwareMap().get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();
    }

    public void setUseWebcam(boolean useWebcam) {
        this.useWebcam = useWebcam;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    @Override
    protected void update() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        if (useWebcam) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    double distanceXMidGoal = detection.ftcPose.x + 25.4;
                    double distanceYMidGoal = detection.ftcPose.y + 25.4;
                    double distanceToGoal = Math.sqrt((distanceXMidGoal * distanceXMidGoal) + (distanceYMidGoal * distanceYMidGoal));
                    if ((SharedData.getAllianceColor() == AllianceSetup.RED && detection.id == 24) || (SharedData.getAllianceColor() == AllianceSetup.BLUE && detection.id == 21)) {
                        SharedData.setDistanceToGoal(distanceToGoal);
                    }

                    double angleTargetRadian = Math.atan2(distanceYMidGoal, distanceXMidGoal);
                    SharedData.setAngleRadToGoal(angleTargetRadian);
                } else {
                    KLog.d("goaldetection", "camera cannot find apriltag");
                }
            }
        } else {
            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());

                telemetry.addData("Botpose", botpose.toString());

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {

                    int tagId = fr.getFiducialId();
                    boolean isRedGoal  = SharedData.getAllianceColor() == AllianceSetup.RED  && tagId == 24;
                    boolean isBlueGoal = SharedData.getAllianceColor() == AllianceSetup.BLUE && tagId == 21;

                    if (isRedGoal || isBlueGoal) {
                        Pose3D tagCamPose = fr.getTargetPoseCameraSpace();

                        double xCam = tagCamPose.getPosition().x * 1000;
                        double yCam = tagCamPose.getPosition().y * 1000;
                        // double zCam = tagCamPose.getPosition().z; //ignore z

                        // 2D distance ignoring height
                        double distance = Math.hypot(xCam, yCam);

                        double angleRad = Math.atan2(yCam, xCam);

                        SharedData.setDistanceToGoal(distance);
                        SharedData.setAngleRadToGoal(angleRad);

                        telemetry.addData("LL GOAL TAG", tagId);
                        telemetry.addData("Goal X (m)", "%.3f", xCam);
                        telemetry.addData("Goal Y (m)", "%.3f", yCam);
                        telemetry.addData("Flat Dist (m)", "%.3f", distance);
                        telemetry.addData("Angle (deg)", "%.1f", Math.toDegrees(angleRad));
                    }

                    // existing debug
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, Xdeg: %.2f, Ydeg: %.2f",
                            fr.getFiducialId(), fr.getFamily(),
                            fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

                telemetry.update();
            } else {
                KLog.d("goaldetection", "limelight cannot find apriltag");
            }
        }
    }
}
