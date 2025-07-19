package com.kalipsorobotics.test.autoMovement;

import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
public class TestAutoDrivePower extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);
        WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        PurePursuitAction purePursuitAction = new PurePursuitAction(driveTrain, wheelOdometry);
        purePursuitAction.addPoint(0, 0, 0);
        purePursuitAction.addPoint(600, 600, 90);
//        purePursuitAction.addPoint(600, -1200, 90);
        double time  = System.currentTimeMillis();

        waitForStart();
        while (opModeIsActive()) {
            if ((System.currentTimeMillis() - time) < 1000) {
                driveTrain.setPower(0.3, 0.3, 0.3, 0.3);
                //driveTrain.setPower(0.3, -0.3, 0.3, -0.3);
            }
        }

    }
}
