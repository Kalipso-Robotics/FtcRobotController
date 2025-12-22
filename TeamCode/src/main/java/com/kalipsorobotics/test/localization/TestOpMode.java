package com.kalipsorobotics.test.localization;

import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KOpMode;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;

//@TeleOp(name = "TestTeleOp", group = "Test")
public class TestOpMode extends KOpMode {
    DriveTrain driveTrain;
    IMUModule imuModule;

    Odometry odometry;
    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        driveTrain = DriveTrain.getInstance(opModeUtilities);
        imuModule = IMUModule.getInstance(opModeUtilities);

        odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        KLog.d("debug_OpMode_Transfer", "TeleOp right " +
                odometry.getRightEncoderMM() +
                " left " + odometry.getLeftEncoderMM() +
                " back " + odometry.getBackEncoderMM() +
                " imu " + odometry.getIMUHeading());


        KLog.d("debug_OpMode_Transfer", "TeleOp " + odometry.toString());

        waitForStart();
        while (opModeIsActive()) {
            KLog.d("debug_OpMode_Transfer", "TeleOp Position " + SharedData.getOdometryIMUPosition());
        }

        cleanupRobot();
    }
}
