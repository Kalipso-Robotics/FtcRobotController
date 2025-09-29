package com.kalipsorobotics.test.cameraVision;

import android.util.Log;

import com.kalipsorobotics.cameraVision.ObiliskDetection;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.GoBildaOdoModule;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.utilities.KMotifDetectionAction;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Autonomous
public class AutoDriveWithMotifDetection extends LinearOpMode {
    KMotifDetectionAction motifDetection;
    ObiliskDetection obiliskDetection;
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        obiliskDetection = new ObiliskDetection();
        obiliskDetection.init(hardwareMap);
        motifDetection = new KMotifDetectionAction(obiliskDetection);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        GoBildaOdoModule goBildaOdoModule = GoBildaOdoModule.getInstance(opModeUtilities);
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, goBildaOdoModule);
        //PurePursuitAction moveForward = new PurePursuitAction(driveTrain);
        //moveForward.addPoint(-500, 0, 0);
        //moveForward.setMaxTimeOutMS(8000);


        ExecutorService executorService = Executors.newSingleThreadExecutor();
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);

        waitForStart();
        while(opModeIsActive()) {
            //moveForward.updateCheckDone();
            driveTrain.setPower(-0.3);
            motifDetection.updateCheckDone();
            Log.d("Motif Detection Auto", "Motif Pattern: " + motifDetection.getMotifPattern());
            telemetry.addLine("" + motifDetection.getMotifPattern());
            if (motifDetection.getIsDone()) {
                driveTrain.setPower(-0.3, 0.3, 0.3, -0.3);
            }
        }
        OpModeUtilities.shutdownExecutorService(executorService);
    }
}
