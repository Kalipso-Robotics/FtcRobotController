package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.checkStuck.checkStuckTest;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.checkStuck.CheckStuckRobot;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.OdometryFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.GoBildaOdoModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Simple autonomous using action-based structure like AutoBasketFunnelWithCheckStuck
 * Instead of KActionSet, uses individual PurePursuitAction instances with stuck detection
 */
@Disabled
public class AutoWithCheckStuck extends LinearOpMode {

    private CheckStuckRobot checkStuckRobot;
    private boolean isCurrentlyStuck = false;
    private PurePursuitAction unstuckAction;
    private boolean isExecutingUnstuckAction;

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);

        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);

        GoBildaOdoModule.setInstanceNull();
        GoBildaOdoModule goBildaOdoModule = GoBildaOdoModule.getInstance(opModeUtilities);

        Odometry.setInstanceNull();
        Odometry wheelOdometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, new Position(0, 0, 0));
        SharedData.resetOdometryWheelIMUPosition();

        // Initialize CheckStuckRobot
        checkStuckRobot = new CheckStuckRobot(driveTrain, wheelOdometry, opModeUtilities, null);

        // Create movement actions - similar to AutoBasketFunnel structure
        PurePursuitAction moveForward1 = new PurePursuitAction(driveTrain);
        moveForward1.addPoint(0, 500, 0); // Move forward 500mm
        moveForward1.setMaxTimeOutMS(8000);

        PurePursuitAction moveForward2 = new PurePursuitAction(driveTrain);
        moveForward2.addPoint(0, 1000, 0); // Move forward to 1000mm total
        moveForward2.setMaxTimeOutMS(8000);

        PurePursuitAction moveBack = new PurePursuitAction(driveTrain);
        moveBack.addPoint(0, 0, 0); // Move back to start
        moveBack.setMaxTimeOutMS(8000);

        telemetry.addLine("init finished");
        telemetry.update();

        ExecutorService executorService = Executors.newSingleThreadExecutor();
        OdometryFileWriter odometryFileWriter = new OdometryFileWriter("SimpleActionBasedAutoWithCheckStuck", opModeUtilities);

        waitForStart();

        OpModeUtilities.runOdometryExecutorService(executorService, wheelOdometry);

        // Action execution state tracking
        boolean action1Done = false;
        boolean action2Done = false;
        boolean action3Done = false;

        while (opModeIsActive()) {
            odometryFileWriter.writeOdometryPositionHistory(SharedData.getOdometryPositionMap());

            // Execute actions in sequence - similar to KActionSet behavior
            if (!isCurrentlyStuck && !isExecutingUnstuckAction) {
                if (!action1Done) {
                    moveForward1.updateCheckDone();
                    if (moveForward1.getIsDone()) {
                        action1Done = true;
                        KLog.d("SimpleActionAuto", "Action 1 completed - moved to 500mm");
                        telemetry.addLine("✅ First movement completed");
                        telemetry.update();
                    }
                } else if (!action2Done) {
                    moveForward2.updateCheckDone();
                    if (moveForward2.getIsDone()) {
                        action2Done = true;
                        KLog.d("SimpleActionAuto", "Action 2 completed - moved to 1000mm");
                        telemetry.addLine("✅ Second movement completed");
                        telemetry.update();
                    }
                } else if (!action3Done) {
                    moveBack.updateCheckDone();
                    if (moveBack.getIsDone()) {
                        action3Done = true;
                        KLog.d("SimpleActionAuto", "Action 3 completed - returned to start");
                        telemetry.addLine("✅ All movements completed!");
                        telemetry.update();
                        break; // All actions complete
                    }
                }
            }

            // Check for stuck condition - identical to original structure
            Position currentPosition = SharedData.getOdometryWheelIMUPosition();
            if (currentPosition != null) {
                boolean isStuck = checkStuckRobot.isStuck(currentPosition);

                if (isStuck && !isCurrentlyStuck) {
                    isCurrentlyStuck = true;
                    KLog.d("SimpleActionBasedAutoWithCheckStuck", "Robot stuck detected - executing recovery");
                    telemetry.addLine("⚠️ ROBOT STUCK - Recovery in progress...");
                    telemetry.update();
                    
                    // Create unstuck action if not already created
                    if (unstuckAction == null) {
                        unstuckAction = new PurePursuitAction(driveTrain);
                        Position safePosition = checkStuckRobot.findBestSafePosition(currentPosition);
                        
                        if (safePosition != null) {
                            KLog.d("unstuck", "Moving to safe position: (" + safePosition.getX() + ", " + safePosition.getY() + ")");
                            unstuckAction.addPoint(safePosition.getX(), safePosition.getY(), safePosition.getTheta());
                        } else {
                            // Fallback: move backward 100mm
                            double backupX = currentPosition.getX() - 100 * Math.cos(currentPosition.getTheta());
                            double backupY = currentPosition.getY() - 100 * Math.sin(currentPosition.getTheta());
                            unstuckAction.addPoint(backupX, backupY, currentPosition.getTheta());
                        }
                        unstuckAction.setMaxTimeOutMS(3000);
                        isExecutingUnstuckAction = true;
                    }
                }
                
                // Execute unstuck action if robot is stuck
                if (isCurrentlyStuck && unstuckAction != null && isExecutingUnstuckAction) {
                    unstuckAction.updateCheckDone();
                    
                    if (unstuckAction.getIsDone()) {
                        KLog.d("SimpleActionBasedAutoWithCheckStuck", "Unstuck action completed");
                        isExecutingUnstuckAction = false;
                        unstuckAction = null;
                        isCurrentlyStuck = false;
                        telemetry.addLine("✅ Recovery completed - continuing auto");
                        telemetry.update();
                    }
                }

                // Update telemetry with current status
                String currentAction = "COMPLETED";
                if (!action1Done) currentAction = "FORWARD 1 (0→500mm)";
                else if (!action2Done) currentAction = "FORWARD 2 (500→1000mm)";
                else if (!action3Done) currentAction = "RETURN (1000→0mm)";
                
                telemetry.addData("Current Action", currentAction);
                telemetry.addData("Status", isCurrentlyStuck ? "RECOVERING" : "RUNNING");
                telemetry.addData("Position", "X: %.1f, Y: %.1f, θ: %.1f°", 
                    currentPosition.getX(), currentPosition.getY(), Math.toDegrees(currentPosition.getTheta()));
                telemetry.update();
            }

            KLog.d("homePos", SharedData.getOdometryWheelIMUPosition().toString());
            KLog.d("homePosMap", SharedData.getOdometryPositionMap().toString());
        }
        
        odometryFileWriter.close();
        OpModeUtilities.shutdownExecutorService(executorService);
    }
}