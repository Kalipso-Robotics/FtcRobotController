package org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * AprilTagCamera backed by the Limelight 3A. Owns the pipeline-per-alliance setup and
 * the Limelight-specific pose-frame conversion; everything downstream only ever sees
 * normalized TagObservations.
 */
public class LimelightAprilTagCamera implements AprilTagCamera {

    private final OpModeUtilities opModeUtilities;
    private final Limelight3A limelight;

    public LimelightAprilTagCamera(OpModeUtilities opModeUtilities, AllianceColor allianceColor) {
        this.opModeUtilities = opModeUtilities;
        limelight = opModeUtilities.getHardwareMap().get(Limelight3A.class, "limelight");
        switchPipeline(allianceColor == AllianceColor.RED ? 0 : 1);
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    @Override
    public List<TagObservation> getLatestObservations() {
        LLResult result = limelight.getLatestResult();
        KLog.d("AprilTag", () -> "AprilTagDetection is running. Is Result valid: " +
                (result != null && result.isValid()) + " Full result:" + result);

        if (result == null || !result.isValid()) {
            return Collections.emptyList();
        }

        List<TagObservation> observations = new ArrayList<>();
        for (LLResultTypes.FiducialResult fiducialResult : result.getFiducialResults()) {
            Pose3D tagRelCamPose = fiducialResult.getTargetPoseCameraSpace();
            Pose3D camRelTagPose = fiducialResult.getCameraPoseTargetSpace();

            double rawPitchDeg = camRelTagPose.getOrientation().getPitch();
            double rawCamPoseX = camRelTagPose.getPosition().x;
            double rawCamPoseZ = camRelTagPose.getPosition().z;

            double camRelTagTheta = MathFunctions.angleWrapRad(Math.toRadians(180 + rawPitchDeg));
            Position camRelTagPos = new Position(-rawCamPoseZ * 1000, -rawCamPoseX * 1000, camRelTagTheta);

            observations.add(new TagObservation(
                    fiducialResult.getFiducialId(),
                    camRelTagPos,
                    rawPitchDeg,
                    tagRelCamPose.getPosition().x * 1000,
                    tagRelCamPose.getPosition().y * 1000,
                    tagRelCamPose.getPosition().z * 1000
            ));
        }
        return observations;
    }

    private void switchPipeline(int index) {
        boolean switched = limelight.pipelineSwitch(index);
        if (!switched) {
            opModeUtilities.getOpMode().sleep(150);
            limelight.pipelineSwitch(index);
        }
    }
}
