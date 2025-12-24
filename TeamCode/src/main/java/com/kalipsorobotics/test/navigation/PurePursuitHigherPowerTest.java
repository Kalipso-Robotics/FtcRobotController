package com.kalipsorobotics.test.navigation;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.autoActions.pathActions.RoundTripAction;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.actions.turret.TurretConfig;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.utilities.KOpMode;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.K;
import org.checkerframework.dataflow.qual.Pure;

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
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule); //3015.93, 765.86, -2.4030
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);


        testAuto = new KActionSet();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        PurePursuitAction purePursuitAction = new PurePursuitAction(driveTrain);
        purePursuitAction.addPoint(400, 0, 90);
        purePursuitAction.addPoint(400, 400, 180);
        //purePursuitAction.addPoint(727, 1150 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        //purePursuitAction.addPoint(2860, 135 * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
        purePursuitAction.setName("testPurePursuit");
        testAuto.addAction(purePursuitAction);

        waitForStart();
        while (opModeIsActive()) {
            testAuto.updateCheckDone();
        }
        cleanupRobot();
    }
}
