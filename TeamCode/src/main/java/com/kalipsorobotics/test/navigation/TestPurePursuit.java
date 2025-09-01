package com.kalipsorobotics.test.navigation;
<<<<<<< HEAD

=======
>>>>>>> eea0ba7fe75cefa00a6f193b683f571f9e2c21ed

import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.localization.OdometryFileWriter;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.GoBildaOdoModule;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Autonomous(name="pptest")
public class TestPurePursuit extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);

        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        GoBildaOdoModule.setInstanceNull();
        GoBildaOdoModule goBildaOdoModule = GoBildaOdoModule.getInstance(opModeUtilities);

        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, goBildaOdoModule);

        ExecutorService executorService = Executors.newSingleThreadExecutor();
        OdometryFileWriter odometryFileWriter = new OdometryFileWriter("AutoBasket", opModeUtilities);

        AdaptivePurePursuitAction test = new AdaptivePurePursuitAction(driveTrain, odometry);
        test.addPoint(0, 0, 0);
        //test.addPoint(50, 0, 0);
        test.addPoint(610, 0, 0);
        test.addPoint(610, -610, 0);
        //test.addPoint(200, 200, 0);
//        test.addPoint(500, 300, 90);
//        test.addPoint(200, 300, 0);
//        test.addPoint(0, 0, 0);

        waitForStart();

        OpModeUtilities.runOdometryExecutorService(executorService, odometry);

        while (opModeIsActive()) {

            odometryFileWriter.writeOdometryPositionHistory(SharedData.getOdometryPositionMap());

            test.updateCheckDone();

            if (test.getIsDone()) {
                break;
            }
        }

        odometryFileWriter.close();
        OpModeUtilities.shutdownExecutorService(executorService);

    }
}