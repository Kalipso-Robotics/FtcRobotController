package com.kalipsorobotics.test.cameraVision;

import com.kalipsorobotics.actions.cameraVision.GoalDetectionAction;
import com.kalipsorobotics.cameraVision.AllianceSetup;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp
public class LimeLightTest extends LinearOpMode {
    OpModeUtilities opModeUtilities;

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        opModeUtilities =  new OpModeUtilities(this.hardwareMap, this, this.telemetry);

        limelight = opModeUtilities.getHardwareMap().get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        waitForStart();

        while (opModeIsActive()) {
            limelight.start();

            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    Pose3D tagCamPose = fr.getTargetPoseCameraSpace();

                    double xCam = tagCamPose.getPosition().x * 1000;
                    double yCam = tagCamPose.getPosition().y * 1000;
                    double zCam = tagCamPose.getPosition().z * 1000;

                    double goalDist3D = Math.sqrt(xCam*xCam + yCam*yCam + zCam*zCam);

                    final double CAM_HEIGHT_M = 12.5 * 25.4; // example: 30 cm off floor
                    final double TAG_HEIGHT_M = 29.5 * 25.4; // example: 40 cm off floor

                    double deltaH = TAG_HEIGHT_M - CAM_HEIGHT_M;

                    double kDistanceScale = (30.0 + 25) / (32 + 27);

                    double flatDist = Math.sqrt(goalDist3D*goalDist3D - deltaH*deltaH) * kDistanceScale; //flat distance on floor with scaling

                    telemetry.addData("LL GOAL TAG", fr.getFiducialId());
                    telemetry.addData("Goal X (offset left-right)", "%.3fmm, %.3fin", xCam, xCam/25.4);
                    telemetry.addData("Goal Z (offset forward-back)", "%.3fmm, %.3fin", zCam, zCam/25.4);
                    telemetry.addData("Goal Flat Distance", "%.3fmm, %.3fin", flatDist, flatDist/25.4);

                    double headingRad = (Math.atan2(xCam, zCam));
                    telemetry.addData("Heading to tag (rad)", "%.3f", headingRad);
//
//                    // existing debug
//                    telemetry.addData("Fiducial", "ID: %d, Xdeg: %.2f, Ydeg: %.2f",
//                            fr.getFiducialId(),
//                            fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

            } else {
                telemetry.addLine("Results invalid.");
            }

            telemetry.update();

        }

        limelight.close();
    }
}
