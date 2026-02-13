package org.firstinspires.ftc.teamcode.kalipsorobotics.test.navigation;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.DrivetrainConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Disabled
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