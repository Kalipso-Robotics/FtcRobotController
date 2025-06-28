package com.kalipsorobotics.odometryDataPaths;

import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.localization.OdometryFileWriter;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@TeleOp (name = "cheese")
public class AutoPathWithHome extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);

        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);

        WheelOdometry.setInstanceNull();
        WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule);
        SharedData.resetOdometryPosition();

        PurePursuitAction moveitmoveit = new PurePursuitAction(driveTrain, wheelOdometry);
//        moveitmoveit.addPoint(0, 0, 0);
//        moveitmoveit.addPoint(-600, 1200, 180);
//        moveitmoveit.addPoint(0, 0, 180);
//        moveitmoveit.addPoint(-600, 0, 180);
//        moveitmoveit.addPoint(0, 0, 90);
//        moveitmoveit.addPoint(0, 800, 90);
//        moveitmoveit.addPoint(-600, 1200, 270);
//        moveitmoveit.addPoint(-600, 300, 90);
//        moveitmoveit.addPoint(-1200, 300, 90);
//        moveitmoveit.addPoint(-1200, 600, 0);
//        moveitmoveit.addPoint(-600, 600, 0);
//        moveitmoveit.addPoint(100, -100, 0, PurePursuitAction.P_XY_SLOW, PurePursuitAction.P_ANGLE_SLOW);
        moveitmoveit.addPoint(0, 0, 0);
        moveitmoveit.addPoint(-600, 1200, 180);
        moveitmoveit.addPoint(0, 0, 180);
        moveitmoveit.addPoint(-600, 300, 180);
        moveitmoveit.addPoint(-1200, 600, 90);
        moveitmoveit.addPoint(0, 800, 0);
        moveitmoveit.addPoint(0,0,0);


        OdometryFileWriter odometryFileWriter = new OdometryFileWriter("TestWithHome", opModeUtilities);
        ExecutorService executorService = Executors.newSingleThreadExecutor();

        waitForStart();
        OpModeUtilities.runOdometryExecutorService(executorService, wheelOdometry);
        while (opModeIsActive()) {
            odometryFileWriter.writeOdometryPositionHistory(SharedData.getOdometryPositionMap());
            moveitmoveit.updateCheckDone();
        }
        odometryFileWriter.close();
        OpModeUtilities.shutdownExecutorService(executorService);

    }
}