package com.kalipsorobotics.test.navigation;

import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.GoBildaOdoModule;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.navigation.PurePursuitFileWriter;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@TeleOp
public class PurePursuitDataCollection extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        DriveAction driveAction = new DriveAction(driveTrain);
        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        GoBildaOdoModule.setInstanceNull();
        GoBildaOdoModule goBildaOdoModule = GoBildaOdoModule.getInstance(opModeUtilities);
        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, goBildaOdoModule);
        PurePursuitAction purePursuitAction = new PurePursuitAction(driveTrain);
        purePursuitAction.addPoint(0, 0, 0);
        purePursuitAction.addPoint(600, 600, 0);
        purePursuitAction.addPoint(1200, -600, 90);
        purePursuitAction.addPoint(1800, -1200, 180);
        purePursuitAction.addPoint(1200, 1200, 0);
        purePursuitAction.addPoint(-600, 0, 90);
        purePursuitAction.addPoint(1800, -600, 180);
        purePursuitAction.addPoint(0, 0, 0);

        PurePursuitFileWriter purePursuitFileWriter = new PurePursuitFileWriter("datafordabigduong", opModeUtilities);

        ExecutorService executorService = Executors.newSingleThreadExecutor();
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);

        waitForStart();
        while (opModeIsActive()) {
            //purePursuitAction.updateCheckDone();
            purePursuitFileWriter.writePurePursuitData(SharedData.getOdometryPositionMap(), driveTrain);
            driveAction.move(gamepad1);
        }
        purePursuitFileWriter.close();

        OpModeUtilities.shutdownExecutorService(executorService);

    }
}
