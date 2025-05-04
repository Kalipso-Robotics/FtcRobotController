package com.kalipsorobotics.test.autoMovement;

import android.util.Log;

import com.kalipsorobotics.utilities.SharedData;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.localization.SparkfunOdometry;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
public class TestPurePursuitAction extends LinearOpMode {

    DriveTrain driveTrain;
    SparkfunOdometry sparkfunOdometry;
    OpModeUtilities opModeUtilities;


    @Override
    public void runOpMode() throws InterruptedException {

        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sparkfunOdometry = new SparkfunOdometry(driveTrain, opModeUtilities, 0, 0, Math.toRadians(0));
        WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule,0,0,Math.toRadians(0));

        PurePursuitAction purePursuitAction = new PurePursuitAction(driveTrain, wheelOdometry);
        purePursuitAction.addPoint(0,0,0);
        purePursuitAction.addPoint(600,600,0);
//        purePursuitAction.addPoint(36,36, Math.toRadians(-90));
//        purePursuitAction.addPoint(24,0, Math.toRadians(-90));
//        purePursuitAction.addPoint(1000,0, 0);
//        purePursuitAction.addPoint(400,-400, 0);

        waitForStart();
        while (opModeIsActive()) {
            wheelOdometry.updateDefaultPosition();
            Log.d("purepursaction_debug_odo_wheel global", SharedData.getOdometryPosition().toString());

//            purePursuitAction.update();
//
//            if (purePursuitAction.checkDoneCondition()) {
//                break;
//            }


        }
    }

}
