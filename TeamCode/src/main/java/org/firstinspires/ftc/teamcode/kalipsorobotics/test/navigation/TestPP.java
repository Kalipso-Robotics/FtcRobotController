package org.firstinspires.ftc.teamcode.kalipsorobotics.test.navigation;

import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.IPurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@TeleOp
public class TestPP extends KOpMode {

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

        IPurePursuitAction test = new AdaptivePurePursuitAction(driveTrain);
        test.addPoint(0,0,0);
        test.addPoint(400,0,90);
        test.addPoint(400,800,180);
        test.addPoint(0,800,0);

        waitForStart();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        opModeUtilities.setAllHubs(allHubs);

        odoExecutorService = Executors.newSingleThreadExecutor();

        OpModeUtilities.runOdometryExecutorService(executorService, odometry);

        while (opModeIsActive()) {

            opModeUtilities.clearBulkCache();
//            OpModeUtilities.runOdometryExecutorService(odoExecutorService, odometry);
            test.updateCheckDone();

            if (test.getIsDone()) {
                break;
            }
        }

        OpModeUtilities.shutdownExecutorService(executorService);

        cleanupRobot();
    }
}