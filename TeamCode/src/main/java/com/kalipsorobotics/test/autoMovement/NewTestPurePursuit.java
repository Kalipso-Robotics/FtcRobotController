package com.kalipsorobotics.test.autoMovement;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.AdaptivePurePursuitAction;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.localization.OdometryFileWriter;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Autonomous(name="pptest")
public class NewTestPurePursuit extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);

        Outtake.setInstanceNull();
        Outtake outtake = Outtake.getInstance(opModeUtilities);

        IntakeClaw.setInstanceNull();
        IntakeClaw intakeClaw = IntakeClaw.getInstance(opModeUtilities);

        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);

        WheelOdometry.setInstanceNull();
        WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        SharedData.resetOdometryPosition();

        //PurePursuitAction test = new PurePursuitAction(driveTrain, wheelOdometry);

        AdaptivePurePursuitAction test = new AdaptivePurePursuitAction(driveTrain, wheelOdometry);
        test.addPoint(0, 0, 0);
        //test.addPoint(50, 0, 0);
        test.addPoint(0, 300, 0);
        //test.addPoint(200, 200, 0);
//        test.addPoint(500, 300, 90);
//        test.addPoint(200, 300, 0);
//        test.addPoint(0, 0, 0);

        ExecutorService executorService = Executors.newSingleThreadExecutor();
        OdometryFileWriter odometryFileWriter = new OdometryFileWriter("AutoBasket", opModeUtilities);

        waitForStart();

        OpModeUtilities.runOdometryExecutorService(executorService, wheelOdometry);

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
