package com.kalipsorobotics.test.navigation;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.utilities.KOpMode;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.K;

@Autonomous(name = "Pure Pursuit Test With Higher Power")
public class PurePursuitHigherPowerTest extends KOpMode {
    KActionSet testAuto;
    IMUModule imuModule = null;
    OpModeUtilities opModeUtilities = null;
    PurePursuitAction purePursuitAction = null;
    DriveTrain driveTrain = null;
    @Override
    public void runOpMode() throws InterruptedException {
        testAuto = new KActionSet();
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        imuModule = IMUModule.getInstance(opModeUtilities);
        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, 3060, 712 * allianceColor.getPolarity(), -2.4049 * allianceColor.getPolarity()); //3015.93, 765.86, -2.4030
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);
        purePursuitAction = new PurePursuitAction(driveTrain);
        purePursuitAction.addPoint(727, 110 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        purePursuitAction.addPoint(727, 1150 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        purePursuitAction.addPoint(2860, 135 * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
        purePursuitAction.setName("testPurePursuit");
        testAuto.addAction(purePursuitAction);

        waitForStart();
        while (opModeIsActive()) {
            testAuto.updateCheckDone();
        }
    }
}
