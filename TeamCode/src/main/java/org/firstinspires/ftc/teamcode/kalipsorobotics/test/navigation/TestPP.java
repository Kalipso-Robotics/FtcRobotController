package org.firstinspires.ftc.teamcode.kalipsorobotics.test.navigation;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;

import android.util.Log;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.IPurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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


        ElapsedTime timer = new ElapsedTime();

        ExecutorService executorService = Executors.newSingleThreadExecutor();
        OpModeUtilities.startOdometry(opModeUtilities, executorService, driveTrain, imuModule, new Position(0, 0, 0));

        IPurePursuitAction test = new AdaptivePurePursuitAction(driveTrain);
        test.addPosition(0,0,0);
        test.addPosition(609.6,0,0);
        test.addPosition(0,0,0);
        test.addPosition(609.6,0,0);
        test.addPosition(609.6,0,180);
        test.addPosition(0,0,0);
        test.addPosition(400,800,180);
        test.addPosition(0,800,0);

        // Same idle-until-START behavior as waitForStart(), but any AdaptivePurePursuitAction
        // built above gets its path planning done during init instead of on match time.
        waitForStartPrecomputingPaths();

        timer.reset();
        Log.d("PPTest", "pure pursuit started");
        telemetry.addLine("pure pursuit started");
        telemetry.update();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        opModeUtilities.setAllHubs(allHubs);

        odoExecutorService = Executors.newSingleThreadExecutor();


        while (opModeIsActive()) {

            opModeUtilities.clearBulkCache();
//            OpModeUtilities.runOdometryExecutorService(odoExecutorService, odometry);
            test.updateCheckDone();

            if (test.getIsDone()) {
                KLog.e("ppDebug", "pp done final ");
                Log.d("ppTest", String.format("Pure pursuit done at %.2f s / %.1f ms", timer.seconds(), timer.milliseconds()));
                Log.d("ppTest", "Finished at location " + SharedData.getOdometryWheelIMUPosition().toCompactString());
                telemetry.addLine(String.format("Pure pursuit done at %.2f s / %.1f ms", timer.seconds(), timer.milliseconds()));
                telemetry.update();

                break;
            }
        }

        OpModeUtilities.shutdownExecutorService(executorService);

        cleanupRobot();
    }
}