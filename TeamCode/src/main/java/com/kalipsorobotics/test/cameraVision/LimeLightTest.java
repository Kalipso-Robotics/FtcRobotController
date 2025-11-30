package com.kalipsorobotics.test.cameraVision;

import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Simplified LimeLight Test - Displays ALL Limelight Data
 *
 * Continuously displays:
 * - ALL Limelight status data
 * - ALL AprilTag fiducial data
 * - Odometry position vs Limelight-calculated position
 * - All camera space, robot space calculations
 *
 * Data is displayed on telemetry and logged via KLog
 */
@TeleOp(name = "LimeLight Test", group = "Test")
public class LimeLightTest extends LinearOpMode {

    private OpModeUtilities opModeUtilities;
    private Limelight3A limelight;
    private Odometry odometry;
    private Turret turret;

    private int frameCount = 0;

    // Physical constants (from GoalDetectionAction)
    private final double APRILTAG_X_FROM_INIT = (15 + 24*4 + 10) * 25.4;
    private final double APRILTAG_Y_FROM_INIT = (7 + 32) * 25.4;
    private final double TURRET_CENTER_TO_LIMELIGHT_DIST = 6.5 * 25.4;
    private final double ROBOT_CENTER_TO_TURRET_DISTANCE = 4.0;
    private final double CAM_HEIGHT_MM = 12.5 * 25.4;
    private final double TAG_HEIGHT_MM = 29.5 * 25.4;
    private final double deltaH = TAG_HEIGHT_MM - CAM_HEIGHT_MM;
    private final double kDistanceScale = (30.0 + 25) / (32 + 27);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("=== LimeLight Test ===");
        telemetry.addLine("Initializing...");
        telemetry.update();

        // Initialize utilities
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        telemetry.addLine("OpModeUtilities: OK");
        telemetry.update();

        // Initialize hardware
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        telemetry.addLine("DriveTrain: OK");
        telemetry.update();

        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        telemetry.addLine("IMUModule: OK");
        telemetry.update();

        // Initialize Odometry
        odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);
        telemetry.addLine("Odometry: OK");
        telemetry.update();

        // Initialize Turret
        turret = Turret.getInstance(opModeUtilities);
        telemetry.addLine("Turret: OK");
        telemetry.update();

        // Initialize Limelight
        limelight = opModeUtilities.getHardwareMap().get(Limelight3A.class, "limelight");
        telemetry.addLine("Limelight: OK");
        telemetry.update();

        telemetry.clear();
        telemetry.addLine("Initialization complete!");
        telemetry.addLine("Press PLAY to start");
        telemetry.update();

        KLog.d("LimeLightTest", "Initialization complete");

        waitForStart();

        // Start Limelight
        limelight.start();

        // Switch to AprilTag pipeline (try pipeline 0, if that doesn't work, try 1, 2, etc.)
        limelight.pipelineSwitch(0);

        KLog.d("LimeLightTest", "Limelight started on pipeline 0");

        telemetry.addLine("Limelight started!");
        telemetry.addLine("If no tags detected:");
        telemetry.addLine("- Point camera at AprilTag");
        telemetry.addLine("- Check Pipeline Type on telemetry");
        telemetry.addLine("- Try pressing DPAD UP/DOWN to switch pipelines");
        telemetry.update();

        sleep(2000); // Give user time to read

        int currentPipeline = 0;
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;

        while (opModeIsActive()) {
            // Update odometry
            if (odometry != null) {
                odometry.update();
            }
            frameCount++;

            // Pipeline switching with DPAD
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;

            if (dpadUp && !lastDpadUp) {
                currentPipeline++;
                if (currentPipeline > 9) currentPipeline = 9; // Max 10 pipelines (0-9)
                limelight.pipelineSwitch(currentPipeline);
                KLog.d("LimeLightTest", "Switched to pipeline " + currentPipeline);
            }

            if (dpadDown && !lastDpadDown) {
                currentPipeline--;
                if (currentPipeline < 0) currentPipeline = 0;
                limelight.pipelineSwitch(currentPipeline);
                KLog.d("LimeLightTest", "Switched to pipeline " + currentPipeline);
            }

            lastDpadUp = dpadUp;
            lastDpadDown = dpadDown;

            // Get Limelight status
            LLStatus status = limelight.getStatus();

            // Get latest result
            LLResult result = limelight.getLatestResult();

            // Clear telemetry
            telemetry.clear();
            telemetry.addLine("=== LimeLight Data ===");
            telemetry.addLine();

            // ========== LIMELIGHT STATUS ==========
            if (status != null) {
                telemetry.addLine("--- LIMELIGHT STATUS ---");
                telemetry.addData("Name", status.getName());
                telemetry.addData("Temperature", "%.1f°C", status.getTemp());
                telemetry.addData("CPU Usage", "%.1f%%", status.getCpu());
                telemetry.addData("FPS", "%.1f", status.getFps());
                telemetry.addData("Pipeline", "%d (%s)", status.getPipelineIndex(), status.getPipelineType());

                // Try to get IP address if available
                try {
                    String ipAddress = limelight.getStatus().getName();
                    if (ipAddress != null && !ipAddress.isEmpty()) {
                        telemetry.addData("Hostname", ipAddress);
                        KLog.d("LL-Status", "Hostname: " + ipAddress);
                    }
                } catch (Exception e) {
                    // IP not available
                }

                KLog.d("LL-Status", String.format("Temp=%.1f°C CPU=%.1f%% FPS=%.1f Pipeline=%d(%s)",
                    status.getTemp(), status.getCpu(), status.getFps(),
                    status.getPipelineIndex(), status.getPipelineType()));
                telemetry.addLine();
            }

            // ========== ODOMETRY POSITION ==========
            Position currentPos = SharedData.getOdometryPosition();
            if (currentPos != null) {
                telemetry.addLine("--- ODOMETRY POSITION ---");
                telemetry.addData("X", "%.1f mm", currentPos.getX());
                telemetry.addData("Y", "%.1f mm", currentPos.getY());
                telemetry.addData("Theta", "%.3f rad (%.1f°)",
                    currentPos.getTheta(), Math.toDegrees(currentPos.getTheta()));

                KLog.d("Odometry", String.format("X=%.1f Y=%.1f Theta=%.3f",
                    currentPos.getX(), currentPos.getY(), currentPos.getTheta()));
                telemetry.addLine();
            }

            // ========== TURRET ANGLE ==========
            if (turret != null) {
                double turretRad = turret.getCurrentAngleRad();
                telemetry.addLine("--- TURRET ---");
                telemetry.addData("Angle", "%.3f rad (%.1f°)",
                    turretRad, Math.toDegrees(turretRad));
                telemetry.addLine();
            }

            // ========== LIMELIGHT RESULT DATA ==========
            telemetry.addLine("--- LIMELIGHT RESULT ---");

            // Check if result exists
            if (result == null) {
                telemetry.addData("Result", "NULL - No data from Limelight");
                KLog.d("LL-Result", "Result is NULL");
            } else {
                // Result exists, check if valid
                boolean isValid = result.isValid();
                telemetry.addData("Result Valid", isValid ? "YES" : "NO");

                // Always show latencies and basic data even if invalid
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("Capture Latency", "%.1f ms", captureLatency);
                telemetry.addData("Targeting Latency", "%.1f ms", targetingLatency);
                telemetry.addData("Parse Latency", "%.1f ms", parseLatency);

                // Crosshair data (always available)
                telemetry.addData("TX", "%.3f°", result.getTx());
                telemetry.addData("TY", "%.3f°", result.getTy());
                telemetry.addData("TXNC", "%.3f", result.getTxNC());
                telemetry.addData("TYNC", "%.3f", result.getTyNC());
                telemetry.addData("TA", "%.3f", result.getTa());

                KLog.d("LL-Result", String.format("Valid=%s TX=%.3f TY=%.3f TA=%.3f Latency=%.1f",
                    isValid ? "YES" : "NO", result.getTx(), result.getTy(), result.getTa(),
                    captureLatency + targetingLatency));

                // Botpose (if available)
                Pose3D botpose = result.getBotpose();
                if (botpose != null && botpose.getPosition() != null) {
                    telemetry.addData("Botpose X", "%.3f", botpose.getPosition().x);
                    telemetry.addData("Botpose Y", "%.3f", botpose.getPosition().y);
                    telemetry.addData("Botpose Z", "%.3f", botpose.getPosition().z);

                    KLog.d("LL-Botpose", String.format("X=%.3f Y=%.3f Z=%.3f",
                        botpose.getPosition().x, botpose.getPosition().y, botpose.getPosition().z));
                } else {
                    telemetry.addData("Botpose", "Not available");
                }

                telemetry.addLine();

                // ========== FIDUCIAL RESULTS ==========
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

                if (fiducialResults != null && !fiducialResults.isEmpty()) {
                    telemetry.addData("Fiducials Found", fiducialResults.size());
                    telemetry.addLine();

                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        processFiducial(fr, currentPos);
                    }
                } else {
                    telemetry.addData("Fiducials", "None detected");
                    if (!isValid) {
                        telemetry.addLine();
                        telemetry.addLine(">> LIKELY ISSUES <<");
                        telemetry.addLine("1. No AprilTags in camera view");
                        telemetry.addLine("2. Wrong pipeline (check pipeline type)");
                        telemetry.addLine("3. AprilTag layout not configured");
                        telemetry.addLine("4. Camera needs to see a tag");
                    }
                }
            }

            telemetry.addData("Frame Count", frameCount);
            telemetry.addLine();
            telemetry.addLine("Controls:");
            telemetry.addLine("  DPAD UP/DOWN: Switch pipeline");
            telemetry.update();
        }

        // Cleanup
        if (limelight != null) {
            limelight.stop();
            KLog.d("LimeLightTest", "Limelight stopped");
        }
        KLog.d("LimeLightTest", "Test complete");
    }

    /**
     * Process and display comprehensive fiducial data
     */
    private void processFiducial(LLResultTypes.FiducialResult fr, Position currentPos) {
        if (fr == null) {
            return;
        }

        try {
            // ========== BASIC FIDUCIAL INFO ==========
            int tagId = fr.getFiducialId();
            String family = fr.getFamily();

            telemetry.addLine("--- TAG " + tagId + " (" + family + ") ---");

            // ========== CAMERA SPACE POSE ==========
            Pose3D tagCamPose = fr.getTargetPoseCameraSpace();
            if (tagCamPose == null || tagCamPose.getPosition() == null) {
                telemetry.addLine("ERROR: Tag camera pose is null");
                KLog.e("LimeLightTest", "Tag " + tagId + " camera pose is null");
                return;
            }

            double xCam = tagCamPose.getPosition().x * 1000;  // Convert to mm
            double yCam = tagCamPose.getPosition().y * 1000;
            double zCam = tagCamPose.getPosition().z * 1000;

            telemetry.addData("Cam X", "%.1f mm", xCam);
            telemetry.addData("Cam Y", "%.1f mm", yCam);
            telemetry.addData("Cam Z", "%.1f mm", zCam);

            // ========== DISTANCES ==========
            double goalDist3D = Math.sqrt(xCam*xCam + yCam*yCam + zCam*zCam);
            double flatDist = Math.sqrt(goalDist3D*goalDist3D - deltaH*deltaH) * kDistanceScale;

            telemetry.addData("3D Distance", "%.1f mm (%.1f in)", goalDist3D, goalDist3D/25.4);
            telemetry.addData("Flat Distance", "%.1f mm (%.1f in)", flatDist, flatDist/25.4);

            // ========== ANGLES ==========
            double headingRad = Math.atan2(xCam, zCam);
            double headingDeg = Math.toDegrees(headingRad);
            telemetry.addData("Heading to Tag", "%.3f rad (%.1f°)", headingRad, headingDeg);

            // ========== TARGET ANGLES & AREA ==========
            double targetXDeg = fr.getTargetXDegrees();
            double targetYDeg = fr.getTargetYDegrees();
            double targetArea = fr.getTargetArea();
            telemetry.addData("Target X", "%.3f°", targetXDeg);
            telemetry.addData("Target Y", "%.3f°", targetYDeg);
            telemetry.addData("Target Area", "%.3f", targetArea);

            // ========== SKEW/ORIENTATION ==========
            double skewRad = 0;
            double skewDeg = 0;
            try {
                if (tagCamPose.getOrientation() != null) {
                    skewRad = tagCamPose.getOrientation().getYaw();
                    skewDeg = Math.toDegrees(skewRad);
                    telemetry.addData("Skew", "%.3f rad (%.1f°)", skewRad, skewDeg);
                }
            } catch (Exception e) {
                telemetry.addData("Skew", "N/A");
            }

            // ========== ROBOT POSE FROM TAG (FIELD SPACE) ==========
            Pose3D robotPoseField = fr.getRobotPoseFieldSpace();
            if (robotPoseField != null && robotPoseField.getPosition() != null) {
                telemetry.addData("Field Robot X", "%.3f", robotPoseField.getPosition().x);
                telemetry.addData("Field Robot Y", "%.3f", robotPoseField.getPosition().y);
                telemetry.addData("Field Robot Z", "%.3f", robotPoseField.getPosition().z);

                KLog.d("LL-FieldRobotPose", String.format("Tag%d X=%.3f Y=%.3f Z=%.3f",
                    tagId, robotPoseField.getPosition().x, robotPoseField.getPosition().y,
                    robotPoseField.getPosition().z));
            }

            // ========== ROBOT POSE FROM TAG (ROBOT/TARGET SPACE) ==========
            Pose3D robotPoseRobot = fr.getRobotPoseTargetSpace();
            if (robotPoseRobot != null && robotPoseRobot.getPosition() != null) {
                telemetry.addData("Target Space X", "%.3f", robotPoseRobot.getPosition().x);
                telemetry.addData("Target Space Y", "%.3f", robotPoseRobot.getPosition().y);
                telemetry.addData("Target Space Z", "%.3f", robotPoseRobot.getPosition().z);

                KLog.d("LL-TargetSpace", String.format("Tag%d X=%.3f Y=%.3f Z=%.3f",
                    tagId, robotPoseRobot.getPosition().x, robotPoseRobot.getPosition().y,
                    robotPoseRobot.getPosition().z));
            }

            // ========== CALCULATED ROBOT POSITION (CUSTOM LOGIC) ==========
            if (currentPos != null && turret != null) {
                double currentTurretRad = turret.getCurrentAngleRad();
                double currentRobotRad = currentPos.getTheta();
                double totalAngleToTag = currentRobotRad + currentTurretRad + headingRad;

                double xDistCamToTag = Math.cos(totalAngleToTag) * flatDist;
                double yDistCamToTag = Math.sin(totalAngleToTag) * flatDist;

                double xDistTurretToLimelight = Math.cos(currentTurretRad) * TURRET_CENTER_TO_LIMELIGHT_DIST;
                double yDistTurretToLimeLight = Math.sin(currentTurretRad) * TURRET_CENTER_TO_LIMELIGHT_DIST;

                double xDistCenterToTurret = Math.cos(currentRobotRad) * ROBOT_CENTER_TO_TURRET_DISTANCE;
                double yDistCenterToTurret = Math.sin(currentRobotRad) * ROBOT_CENTER_TO_TURRET_DISTANCE;

                double totalX = xDistCamToTag + xDistTurretToLimelight + xDistCenterToTurret;
                double totalY = yDistCamToTag + yDistTurretToLimeLight + yDistCenterToTurret;

                double limelightCalcX = APRILTAG_X_FROM_INIT - totalX;
                double limelightCalcY = APRILTAG_Y_FROM_INIT - totalY;

                telemetry.addLine();
                telemetry.addLine("--- POSITION COMPARISON ---");
                telemetry.addData("Odometry X", "%.1f mm", currentPos.getX());
                telemetry.addData("Limelight Calc X", "%.1f mm", limelightCalcX);
                telemetry.addData("Delta X", "%.1f mm", limelightCalcX - currentPos.getX());
                telemetry.addLine();
                telemetry.addData("Odometry Y", "%.1f mm", currentPos.getY());
                telemetry.addData("Limelight Calc Y", "%.1f mm", limelightCalcY);
                telemetry.addData("Delta Y", "%.1f mm", limelightCalcY - currentPos.getY());

                KLog.d("LL-Comparison", String.format("Tag%d | Odo(%.1f,%.1f) LL(%.1f,%.1f) Delta(%.1f,%.1f)",
                    tagId, currentPos.getX(), currentPos.getY(),
                    limelightCalcX, limelightCalcY,
                    limelightCalcX - currentPos.getX(), limelightCalcY - currentPos.getY()));
            }

            // ========== COMPREHENSIVE KLOG ==========
            KLog.d("LL-Tag" + tagId, String.format(
                "Cam(%.1f,%.1f,%.1f) 3D=%.1f Flat=%.1f Head=%.1f° TX=%.3f° TY=%.3f° Area=%.3f",
                xCam, yCam, zCam, goalDist3D, flatDist, headingDeg,
                targetXDeg, targetYDeg, targetArea));

            telemetry.addLine();

        } catch (Exception e) {
            KLog.e("LimeLightTest", "Error processing fiducial: " + e.getMessage());
            telemetry.addData("Error", e.getMessage());
        }
    }
}
