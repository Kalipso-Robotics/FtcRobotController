package com.kalipsorobotics.test.navigation;

import com.kalipsorobotics.decode.configs.DrivetrainConfig;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.utilities.KOpMode;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@TeleOp
public class TestPurePursuit extends KOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        super.initializeRobot();
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);

        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);

        ExecutorService executorService = Executors.newSingleThreadExecutor();

        PurePursuitAction test = new PurePursuitAction(driveTrain);
//        test.addPoint(0, 0, 0);
//        test.addPoint(1200, 0, 0);
//        test.addPoint(1200, 1200, 90);
//        test.addPoint(600, 1200, 90);

        test.addPoint(1422.4, -508, -90);
        test.addPoint(1422.4, -1219.2, -90);
        test.addPoint(0, 0, 0);



        waitForStart();

        OpModeUtilities.runOdometryExecutorService(executorService, odometry);

        while (opModeIsActive()) {


            test.updateCheckDone();

            if (test.getIsDone()) {
                break;
            }
        }

        OpModeUtilities.shutdownExecutorService(executorService);

    }
}