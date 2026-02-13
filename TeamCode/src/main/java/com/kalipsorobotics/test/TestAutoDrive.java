package com.kalipsorobotics.test;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KOpMode;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Disabled
@Autonomous
public class TestAutoDrive extends KOpMode {

    KActionSet testAutoDrive;
    private DriveTrain driveTrain;

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.RED;
        SharedData.setAllianceColor(allianceColor);
    }

    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        KLog.d("RedAutoDepot-Init", "Starting initializeRobot()");

        // Create your modules
        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);

        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000); // Optional: let hardware initialize

        // Create odometry
        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);

        testAutoDrive = new KActionSet();



        KLog.d("RedAutoDepot-Init", "Finished initializeRobot()");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        PurePursuitAction testDrive = new PurePursuitAction(driveTrain);
        testDrive.addPoint(0, 0, 0);
        testDrive.addPoint(600, 0, 0);
        testDrive.addPoint(0, 300, 90);


        waitForStart();
        while (opModeIsActive()) {

            testDrive.updateCheckDone();

        }
        cleanupRobot();
    }


}
