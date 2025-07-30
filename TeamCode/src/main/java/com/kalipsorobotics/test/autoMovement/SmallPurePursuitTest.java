package com.kalipsorobotics.test.autoMovement;

import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name="straightTest")
public class SmallPurePursuitTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);
        WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        PurePursuitAction test = new PurePursuitAction(driveTrain, wheelOdometry);
        test.addPoint(0, 0, 0);
        test.addPoint(0,254,0);
        sleep(2000);
        test.addPoint(254,254,0);
        sleep(2000);
        test.addPoint(254,254,180);


        waitForStart();
        while (opModeIsActive()) {
            wheelOdometry.updateDefaultPosition();
            test.update();
            if (test.checkDoneCondition()) {
                break;
            }
        }

    }
}
