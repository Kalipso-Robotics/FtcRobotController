package com.kalipsorobotics.test.cameraVision;

import com.kalipsorobotics.localization.LimelightLocalization;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Test Program for LimelightLocalization
 *
 * Demonstrates how to use the new LimelightLocalization class:
 * - Initialize Limelight and Odometry
 * - Detect AprilTags
 * - Correct odometry position
 * - Get distance and angle to targets
 *
 * Controls:
 * - GAMEPAD1 BACK: Correct odometry from vision
 * - GAMEPAD1 X: Display visible tags
 * - GAMEPAD1 Y: Display distance to tag 11
 * - GAMEPAD1 A: Start Limelight
 * - GAMEPAD1 B: Stop Limelight
 */
@TeleOp(name = "Limelight Localization Test", group = "Test")
public class LimelightLocalizationTest extends LinearOpMode {

    private LimelightLocalization limelight;
    private Odometry odometry;
    private OpModeUtilities opModeUtilities;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("=== Limelight Localization Test ===");
        telemetry.addLine("Initializing...");
        telemetry.update();

        // Initialize hardware
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        // Initialize Odometry with a test starting position
        odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule,
            1000, 1000, 0);  // Start at (1000mm, 1000mm, 0rad)

        // Initialize Limelight
        try {
            limelight = LimelightLocalization.getInstance(opModeUtilities, odometry);
            telemetry.addLine("Limelight initialized successfully!");
        } catch (Exception e) {
            telemetry.addLine("ERROR: Failed to initialize Limelight");
            telemetry.addData("Error", e.getMessage());
            telemetry.update();

            waitForStart();
            while (opModeIsActive()) {
                telemetry.addLine("Limelight not available - exiting");
                telemetry.update();
                sleep(1000);
            }
            return;
        }

        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  BACK: Correct odometry");
        telemetry.addLine("  X: Show visible tags");
        telemetry.addLine("  Y: Distance to tag 11");
        telemetry.addLine("  A: Start Limelight");
        telemetry.addLine("  B: Stop Limelight");
        telemetry.addLine();
        telemetry.addLine("Press PLAY to start");
        telemetry.update();

        KLog.d("LimelightTest", "Initialization complete");

        waitForStart();

        // Start Limelight by default
        limelight.start();

        while (opModeIsActive()) {
            // Read buttons
            boolean correctOdometry = gamepad1.back;
            boolean showTags = gamepad1.x;
            boolean showDistance = gamepad1.y;
            boolean startLimelight = gamepad1.a;
            boolean stopLimelight = gamepad1.b;

            // Update odometry
            Position currentPos = SharedData.getOdometryPosition();

            // Clear telemetry
            telemetry.clear();
            telemetry.addLine("=== Limelight Localization Test ===");
            telemetry.addLine();

            // Display current odometry position
            telemetry.addData("Odometry X", "%.1f mm", currentPos.getX());
            telemetry.addData("Odometry Y", "%.1f mm", currentPos.getY());
            telemetry.addData("Odometry Theta", "%.2f rad (%.1f°)",
                currentPos.getTheta(), Math.toDegrees(currentPos.getTheta()));
            telemetry.addLine();

            // Handle Limelight start/stop
            if (startLimelight) {
                limelight.start();
                telemetry.addLine("Limelight STARTED");
            }
            if (stopLimelight) {
                limelight.stop();
                telemetry.addLine("Limelight STOPPED");
            }

            // Handle odometry correction
            if (correctOdometry) {
                telemetry.addLine(">>> CORRECTING ODOMETRY <<<");
                boolean success = limelight.updateOdometry();

                if (success) {
                    gamepad1.rumble(200);  // Success feedback
                    Position newPos = SharedData.getOdometryPosition();
                    telemetry.addData("Correction", "SUCCESS");
                    telemetry.addData("New X", "%.1f mm", newPos.getX());
                    telemetry.addData("New Y", "%.1f mm", newPos.getY());
                    telemetry.addData("New Theta", "%.2f rad", newPos.getTheta());

                    int[] tags = limelight.getVisibleTagIds();
                    if (tags.length > 0) {
                        telemetry.addData("Used Tag", tags[0]);
                    }
                } else {
                    gamepad1.rumble(100);  // Failure feedback
                    telemetry.addData("Correction", "FAILED - No tags visible");
                }
                telemetry.addLine();
            }

            // Show visible tags
            if (showTags || limelight.hasTarget()) {
                int[] visibleTags = limelight.getVisibleTagIds();

                if (visibleTags.length > 0) {
                    telemetry.addData("Visible Tags", java.util.Arrays.toString(visibleTags));

                    // Show details for each tag
                    for (int tagId : visibleTags) {
                        double distance = limelight.getDistanceToTarget(tagId);
                        double angle = limelight.getAngleToTarget(tagId);

                        telemetry.addData("Tag " + tagId + " Distance", "%.1f mm (%.1f in)",
                            distance, distance / 25.4);
                        telemetry.addData("Tag " + tagId + " Angle", "%.2f rad (%.1f°)",
                            angle, Math.toDegrees(angle));
                    }
                } else {
                    telemetry.addData("Visible Tags", "None");
                }
                telemetry.addLine();
            }

            // Show distance to specific tag
            if (showDistance) {
                double distance = limelight.getDistanceToTarget(11);  // Tag 11
                double angle = limelight.getAngleToTarget(11);

                if (distance >= 0) {
                    telemetry.addData("Tag 11 Distance", "%.1f mm (%.1f in)",
                        distance, distance / 25.4);
                    telemetry.addData("Tag 11 Angle", "%.2f rad (%.1f°)",
                        angle, Math.toDegrees(angle));
                } else {
                    telemetry.addData("Tag 11", "Not visible");
                }
                telemetry.addLine();
            }

            // Control hints
            telemetry.addLine("Controls:");
            telemetry.addLine("  BACK=Correct | X=Tags | Y=Dist");
            telemetry.addLine("  A=Start | B=Stop");

            telemetry.update();
            sleep(50);  // Reduce CPU usage
        }

        // Cleanup
        limelight.stop();
        KLog.d("LimelightTest", "Test complete");
    }
}
