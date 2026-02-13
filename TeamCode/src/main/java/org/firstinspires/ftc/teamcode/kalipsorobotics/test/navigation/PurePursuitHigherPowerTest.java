package org.firstinspires.ftc.teamcode.kalipsorobotics.test.navigation;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.cameraVision.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Pure Pursuit Test With Higher Power")
public class PurePursuitHigherPowerTest extends KOpMode {
    KActionSet testAuto;

    public DriveTrain driveTrain;

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.RED;
        SharedData.setAllianceColor(allianceColor);
    }

    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        // Create your modules
        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000); // Optional: let hardware initialize

        // Create odometry
        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, 3060, 712 * allianceColor.getPolarity(), -2.4049 * allianceColor.getPolarity()); //3015.93, 765.86, -2.4030
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);

        testAuto = new KActionSet();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        PurePursuitAction purePursuitAction1 = new PurePursuitAction(driveTrain);
//        purePursuitAction.addPoint(400, 0, 90);
//        purePursuitAction.addPoint(400, 400, 180);
//        purePursuitAction.addPoint(800, 800, 0);
//        purePursuitAction.addPoint(400, 800, 90);
        purePursuitAction1.addPoint(2598, 441.38, -138.29);
        testAuto.addAction(purePursuitAction1);

        PurePursuitAction purePursuitAction2 = new PurePursuitAction(driveTrain);
        purePursuitAction2.addPoint(1970, 175, 90);
        purePursuitAction2.addPoint(1970, 840, 90);
        // move to hit lever
        purePursuitAction2.addPoint(1755, 1100, 0);
        // move to launch
        purePursuitAction2.addPoint(2130, 135, 180);
        purePursuitAction2.setDependentActions(purePursuitAction1);
        testAuto.addAction(purePursuitAction2);

        PurePursuitAction purePursuitAction3 = new PurePursuitAction(driveTrain);
        purePursuitAction3.addPoint(1300, 200, 90);
        purePursuitAction3.addPoint(1300, 1100, 90);
        purePursuitAction3.addPoint(1300, 870, 90);
        purePursuitAction3.addPoint(2130, 135, 180);
        purePursuitAction3.setDependentActions(purePursuitAction2);
//        purePursuitAction.setName("testPurePursuit");
        testAuto.addAction(purePursuitAction3);

        waitForStart();
        while (opModeIsActive()) {
            testAuto.updateCheckDone();
        }
        cleanupRobot();
    }
}
