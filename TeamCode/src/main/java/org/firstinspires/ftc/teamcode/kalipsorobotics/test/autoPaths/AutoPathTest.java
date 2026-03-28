package org.firstinspires.ftc.teamcode.kalipsorobotics.test.autoPaths;

import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

public class AutoPathTest extends KOpMode {
    DriveTrain driveTrain = null;
    PurePursuitAction purePursuit = null;

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
        OpModeUtilities.runOdometryExecutorService(odoExecutorService, odometry);
    }
    @Override
    public void runOpMode() throws InterruptedException {


    }
}
