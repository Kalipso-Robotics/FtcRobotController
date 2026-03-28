package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;

import java.util.Optional;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

@Autonomous
public class AdaptivePurePursuitTest extends KOpMode {

    public DriveTrain driveTrain;
    private Odometry odometry;
    private AdaptivePurePursuitAction adaptivePP;

    // Target endpoint
    private Position targetPosition;

    // Key metrics (like delta RPS for shooter, delta angle for turret)
    private double maxPositionError = 0;      // Worst deviation from target during run (mm)
    private double avgPositionError = 0;      // Average error during run (mm)
    private double finalPositionError = 0;    // Final endpoint error (mm)
    private int sampleCount = 0;
    private int logCounter = 0;               // Counter for periodic logging

    // Performance target
    private static final double TARGET_TIME_SEC = 3.5;  // Target time to complete path

    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);

        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        Odometry.setInstanceNull();
        // Initialize odometry to blue starting position (3060, -712, 2.4049 rad)
        odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, 3060, -712, 2.4049);
        OpModeUtilities.runOdometryExecutorService(odoExecutorService, odometry);

        adaptivePP = new AdaptivePurePursuitAction(driveTrain, odometry);
        adaptivePP.setName("AdaptivePP_Test");

        // Blue path - all waypoints from RedAutoNearRampThirdSpike
        adaptivePP.addPoint(2400, -128, 138.29);  // First shoot
        // 2nd spike
        adaptivePP.addPoint(1400, -225, -90);
        adaptivePP.addPoint(1400, -1025, -90);
        adaptivePP.addPoint(1500, 0, -150);
        adaptivePP.addPoint(1850, 0, -150);
        // ramp
        adaptivePP.addPoint(1340, -825, -53);
        adaptivePP.addPoint(1350, -1045, -77);
        adaptivePP.addPoint(1509, -942, -90);
        adaptivePP.addPoint(1850, 0, -150);

        targetPosition = new Position(1850, 0, Math.toRadians(-150));

        // Log test configuration
        KLog.d("AdaptivePPTest", "========== TEST CONFIGURATION ==========");
        KLog.d("AdaptivePPTest", String.format("Starting Position: (3060, -712, 2.4049 rad)"));
        KLog.d("AdaptivePPTest", String.format("Path points: %d", 9));
        KLog.d("AdaptivePPTest", String.format("Target: (%.1f, %.1f, %.1f°)",
            targetPosition.getX(), targetPosition.getY(), Math.toDegrees(targetPosition.getTheta())));
        KLog.d("AdaptivePPTest", String.format("Target Time: %.1f sec", TARGET_TIME_SEC));
        KLog.d("AdaptivePPTest", "========================================");

        telemetry.addLine("AdaptivePurePursuit Test Ready");
        telemetry.addData("Target", targetPosition.toCompactString());
        telemetry.addData("Target Time", "%.1f sec", TARGET_TIME_SEC);
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        double errorSum = 0;

        KLog.d("AdaptivePPTest", "========== PATH EXECUTION START ==========");
        KLog.d("AdaptivePPTest", String.format("Target: (%.1f, %.1f, %.1f°)",
            targetPosition.getX(), targetPosition.getY(), Math.toDegrees(targetPosition.getTheta())));

        // Run path
        while (opModeIsActive() && !adaptivePP.getIsDone()) {
            adaptivePP.updateCheckDone();

            Position currentPos = SharedData.getOdometryWheelIMUPosition();

            // Measure error to closest point on path (cross-track error)
            // This is the real path-following error, not distance to lookahead point
            Optional<Position> closestPoint = adaptivePP.getClosestPathPoint();
            Optional<Position> followPoint = adaptivePP.getCurrentFollowPoint();
            double positionError = 0;

            if (closestPoint.isPresent()) {
                positionError = currentPos.distanceTo(closestPoint.get());

                // Track metrics (like tracking delta RPS for shooter)
                if (positionError > maxPositionError) {
                    maxPositionError = positionError;
                }
                errorSum += positionError;
                sampleCount++;

                // Log data every 10 samples for analysis
                if (logCounter % 10 == 0) {
                    String followStr = followPoint.isPresent() ?
                        String.format("(%.1f,%.1f)", followPoint.get().getX(), followPoint.get().getY()) :
                        "NONE";

                    KLog.d("AdaptivePPTest", String.format(
                        "t=%.2fs | Pos:(%.1f,%.1f,%.1f°) | Closest:(%.1f,%.1f) | Follow:%s | Err:%.1fmm | Max:%.1fmm",
                        timer.seconds(),
                        currentPos.getX(), currentPos.getY(), Math.toDegrees(currentPos.getTheta()),
                        closestPoint.get().getX(), closestPoint.get().getY(),
                        followStr,
                        positionError,
                        maxPositionError
                    ));
                }
                logCounter++;
            } else {
                // During initialization phase - no path yet
                if (logCounter % 50 == 0) {
                    KLog.d("AdaptivePPTest", String.format(
                        "t=%.2fs | INITIALIZING PATH",
                        timer.seconds()
                    ));
                }
                logCounter++;
            }

            // Display real-time
            telemetry.addData("Time", "%.1f / %.1f sec", timer.seconds(), TARGET_TIME_SEC);
            telemetry.addData("Current Pos", currentPos.toCompactString());
            telemetry.addData("Status", closestPoint.isPresent() ? "Following" : "Initializing");
            if (closestPoint.isPresent()) {
                telemetry.addData("Error", "%.1f mm (%.2f in)", positionError, positionError / 25.4);
                telemetry.addData("Max Error", "%.1f mm", maxPositionError);
            }
            telemetry.update();
        }

        // Calculate final metrics
        Position finalPos = SharedData.getOdometryWheelIMUPosition();
        finalPositionError = finalPos.distanceTo(targetPosition);
        avgPositionError = sampleCount > 0 ? errorSum / sampleCount : 0;
        double finalTime = timer.seconds();

        // Log key metrics (like you do for shooter/turret)
        KLog.d("AdaptivePPTest", "========== TEST RESULTS ==========");
        KLog.d("AdaptivePPTest", String.format("Target Position: (%.1f, %.1f, %.1f°)",
            targetPosition.getX(), targetPosition.getY(), Math.toDegrees(targetPosition.getTheta())));
        KLog.d("AdaptivePPTest", String.format("Final Position: (%.1f, %.1f, %.1f°)",
            finalPos.getX(), finalPos.getY(), Math.toDegrees(finalPos.getTheta())));
        KLog.d("AdaptivePPTest", "---");
        KLog.d("AdaptivePPTest", String.format("Final Error: %.1f mm (%.2f in)",
            finalPositionError, finalPositionError / 25.4));
        KLog.d("AdaptivePPTest", String.format("Max Error During Run: %.1f mm (%.2f in)",
            maxPositionError, maxPositionError / 25.4));
        KLog.d("AdaptivePPTest", String.format("Avg Error During Run: %.1f mm (%.2f in)",
            avgPositionError, avgPositionError / 25.4));
        KLog.d("AdaptivePPTest", "---");
        KLog.d("AdaptivePPTest", String.format("Time: %.2f sec (Target: %.1f sec)", finalTime, TARGET_TIME_SEC));
        KLog.d("AdaptivePPTest", String.format("Sample Count: %d", sampleCount));
        KLog.d("AdaptivePPTest", String.format("Avg Loop Time: %.2f ms", (finalTime * 1000.0) / sampleCount));

        // Simple pass/fail (like "3% error RPS at all times")
        boolean passAccuracy = finalPositionError < 25.4;  // <1 inch
        boolean passConsistency = maxPositionError < 100;  // <100mm worst case during run
        boolean passFastEnough = finalTime <= TARGET_TIME_SEC;
        KLog.d("AdaptivePPTest", "---");
        KLog.d("AdaptivePPTest", String.format("Accuracy (<1 inch final): %s (%.1f mm)",
            passAccuracy ? "PASS" : "FAIL", finalPositionError));
        KLog.d("AdaptivePPTest", String.format("Consistency (<100mm max): %s (%.1f mm)",
            passConsistency ? "PASS" : "FAIL", maxPositionError));
        KLog.d("AdaptivePPTest", String.format("Speed (<%.1fs): %s (%.2f sec)",
            TARGET_TIME_SEC, passFastEnough ? "PASS" : "FAIL", finalTime));
        KLog.d("AdaptivePPTest", String.format("OVERALL: %s",
            (passAccuracy && passConsistency && passFastEnough) ? "PASS ✓" : "FAIL ✗"));
        KLog.d("AdaptivePPTest", "========================================");

        // Display on driver station
        while (opModeIsActive()) {
            telemetry.addLine("========== RESULTS ==========");
            telemetry.addData("Final Error", "%.1f mm (%.2f in)",
                finalPositionError, finalPositionError / 25.4);
            telemetry.addData("Max Error", "%.1f mm", maxPositionError);
            telemetry.addData("Avg Error", "%.1f mm", avgPositionError);
            telemetry.addData("Time", "%.2f sec", finalTime);
            telemetry.addLine();
            telemetry.addData("Accuracy (<1 inch)", passAccuracy ? "PASS ✓" : "FAIL ✗");
            telemetry.addData("Consistency (<100mm)", passConsistency ? "PASS ✓" : "FAIL ✗");
            telemetry.addData("Fast Enough (<%.1fs)", passFastEnough ? "PASS ✓" : "FAIL ✗");
            telemetry.addLine();
            telemetry.addData("OVERALL", (passAccuracy && passConsistency && passFastEnough) ? "PASS ✓" : "FAIL ✗");
            telemetry.update();
        }
    }
}
