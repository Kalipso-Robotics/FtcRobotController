package com.kalipsorobotics.test.localization;

import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KOpMode;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;

//@Autonomous
public class TestAuto extends KOpMode {
    DriveTrain driveTrain;
    IMUModule imuModule;

    Odometry odometry;
    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        imuModule = IMUModule.getInstance(opModeUtilities);

        Odometry.setInstanceNull();
        odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        PurePursuitAction purePursuitAction = new PurePursuitAction(driveTrain);
        purePursuitAction.addPoint(400, 400, 90);

        waitForStart();
        while (opModeIsActive()) {
            purePursuitAction.updateCheckDone();
            KLog.d("debug_OpMode_Transfer", "Auto Position " + SharedData.getOdometryIMUPosition());
        }
        KLog.d("debug_OpMode_Transfer", "Auto End right (supposed to be 0)" +
                odometry.getRightEncoderMM() +
                " left " + odometry.getLeftEncoderMM() +
                " back " + odometry.getBackEncoderMM() +
                " imu " + odometry.getIMUHeading());


        KLog.d("debug_OpMode_Transfer", "Auto End " + odometry.toString());
        cleanupRobot();
        KLog.d("debug_OpMode_Transfer", "Auto After Cleanup right " +
                odometry.getRightEncoderMM() +
                " left " + odometry.getLeftEncoderMM() +
                " back " + odometry.getBackEncoderMM() +
                " imu " + odometry.getIMUHeading());


        KLog.d("debug_OpMode_Transfer", "Auto After Cleanup " + odometry.toString());
    }
}
