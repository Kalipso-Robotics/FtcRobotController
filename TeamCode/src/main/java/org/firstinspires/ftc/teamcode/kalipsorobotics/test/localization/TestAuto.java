package org.firstinspires.ftc.teamcode.kalipsorobotics.test.localization;

import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

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
            KLog.d("debug_OpMode_Transfer", "Auto Position " + SharedData.getOdometryWheelIMUPosition());
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
