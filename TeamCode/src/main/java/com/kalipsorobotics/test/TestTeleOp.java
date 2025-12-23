package com.kalipsorobotics.test;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KOpMode;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class TestTeleOp extends KOpMode {

    KActionSet testAutoDrive;
    private DriveTrain driveTrain;
    DriveAction driveAction = null;

    boolean drivingSticks;

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.RED;
        SharedData.setAllianceColor(allianceColor);
    }

    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        KLog.d("RedAutoDepot-Init", "Starting initializeRobot()");

        // Reuse DriveTrain instance from Auto (don't call setInstanceNull!)
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        driveAction = new DriveAction(driveTrain);

        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000); // Optional: let hardware initialize

        // Reuse odometry instance from Auto (don't call setInstanceNull!)
        KLog.d("Auto→TeleOp", "=== TELEOP STARTING ===");
        KLog.d("Auto→TeleOp", "Before getInstance: " + SharedData.getOdometryWheelIMUPosition());
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);
        KLog.d("Auto→TeleOp", "After getInstance: " + SharedData.getOdometryWheelIMUPosition());
        KLog.d("Auto→TeleOp", "Odometry instance: " + System.identityHashCode(odometry));
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);

        testAutoDrive = new KActionSet();

        KLog.d("RedAutoDepot-Init", "Finished initializeRobot()");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        KLog.d("Auto→TeleOp", "After initializeRobot: " + SharedData.getOdometryWheelIMUPosition());
        waitForStart();
        sleep(50);
        //Wait for Executor Thread to start
        KLog.d("Auto→TeleOp", "After waitForStart: " + SharedData.getOdometryWheelIMUPosition());
        while (opModeIsActive()) {

            drivingSticks = kGamePad1.getLeftStickY() != 0 ||
                    kGamePad1.getRightStickX() != 0 ||
                    kGamePad1.getLeftStickX() != 0;

            if (drivingSticks) {

                driveAction.move(kGamePad1.getGamepad());
            } else {
                driveTrain.setPower(0);
            }
        }
        cleanupRobot();
    }


}
