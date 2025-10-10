package com.kalipsorobotics.test.navigation;

import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.Vector;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.GoBildaOdoModule;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AdaptivePurePursuitRobotData extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);

        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        GoBildaOdoModule.setInstanceNull();
        GoBildaOdoModule goBildaOdoModule = GoBildaOdoModule.getInstance(opModeUtilities);

        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);

        Position prevPosition = new Position(0,0,0);
        double prevTime = 0;
        ElapsedTime elapsedTime = new ElapsedTime();
        double prevLinearVelocity = 0;

        double maxLinearVelocity = 0;
        double maxLinearAcceleration = 0;
        double maxAngularVelocity = 0;

        waitForStart();

        while (opModeIsActive()) {
            Position currentPosition = SharedData.getOdometryPosition();
            Vector deltaPosition = Vector.between(prevPosition, currentPosition);

            double currentTime = elapsedTime.seconds();
            double deltaTime = currentTime - prevTime;

            double currentLinearVelocity = deltaPosition.getLength() / deltaTime; // mm/s

            if (currentLinearVelocity > maxLinearVelocity) {
                maxLinearAcceleration = currentLinearVelocity;
            }

            double deltaLinearVelocity = currentLinearVelocity - prevLinearVelocity;
            double linearAcceleration = deltaLinearVelocity / deltaTime;

            if (linearAcceleration > maxLinearAcceleration) {
                maxLinearAcceleration = linearAcceleration;
            }

            double deltaRad = currentPosition.getTheta() - prevPosition.getTheta();
            double angularVelocity = deltaRad / deltaTime; // rad/s

            if (angularVelocity > maxAngularVelocity) {
                maxAngularVelocity = angularVelocity;
            }
        }

        KLog.d("AdaptivePurePursuitRobotData", "maxLinearAcceleration: " + maxLinearAcceleration);
        KLog.d("AdaptivePurePursuitRobotData", "maxLinearVelocity: " + maxLinearVelocity);
        KLog.d("AdaptivePurePursuitRobotData", "maxAngularVelocity: " + maxAngularVelocity);

    }
}
