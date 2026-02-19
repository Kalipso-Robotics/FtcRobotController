package org.firstinspires.ftc.teamcode.kalipsorobotics.test.localization;

import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

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
        OpModeUtilities.runOdometryExecutorService(odoExecutorService, odometry);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        KLog.d("debug_OpMode_Transfer", () -> "TeleOp right " +
                odometry.getRightEncoderMM() +
                " left " + odometry.getLeftEncoderMM() +
                " back " + odometry.getBackEncoderMM() +
                " imu " + odometry.getIMUHeading());


        KLog.d("debug_OpMode_Transfer", () -> "TeleOp " + odometry.toString());

        waitForStart();
        while (opModeIsActive()) {
            KLog.d("debug_OpMode_Transfer", () -> "TeleOp Position " + SharedData.getOdometryWheelIMUPosition());
        }

        cleanupRobot();
    }
}
