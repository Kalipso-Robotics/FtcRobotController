package org.firstinspires.ftc.teamcode.kalipsorobotics.test.navigation;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.DriveAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Vector;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.GoBildaOdoModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;


@TeleOp
public class AdaptivePurePursuitRobotData extends KOpMode {
    @Override
    public void runOpMode() throws InterruptedException { //work
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);


        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);

        ExecutorService executorService = Executors.newSingleThreadExecutor();

        ElapsedTime elapsedTime = new ElapsedTime();
        Position prevPosition = null;
        double prevTime = 0;
        double prevLinearVelocity = 0;

        double maxLinearVelocity = 0;
        double maxLinearAcceleration = 0;
        double maxAngularVelocity = 0;
        double smoothedVelocity = 0;
        final double VELOCITY_SMOOTHING = 0.7;
        double accelWindowVelocity = 0;
        double accelWindowTime = 0;
        final double ACCEL_WINDOW_S = 0.2; // compute acceleration over 200ms to avoid derivative noise

        DriveAction drive = new DriveAction(driveTrain);

        waitForStart();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        opModeUtilities.setAllHubs(allHubs);

        odoExecutorService = Executors.newSingleThreadExecutor();

        OpModeUtilities.runOdometryExecutorService(executorService, odometry);

        while (opModeIsActive()) {

            opModeUtilities.clearBulkCache();


            drive.move(gamepad1);

            Position currentPosition = SharedData.getOdometryWheelIMUPosition();
            double currentTime = elapsedTime.seconds();

            if (prevPosition == null) {
                // skip first iteration — deltaTime would be invalid
                prevPosition = currentPosition;
                prevTime = currentTime;
                continue;
            }

            double deltaTime = currentTime - prevTime;
            if (deltaTime < 0.02) { // require 20ms between samples to avoid noise from tiny deltaTime
                continue;
            }

            Vector deltaPosition = Vector.between(prevPosition, currentPosition);
            double currentLinearVelocity = deltaPosition.getLength() / deltaTime; // mm/s
            smoothedVelocity = VELOCITY_SMOOTHING * smoothedVelocity + (1 - VELOCITY_SMOOTHING) * currentLinearVelocity;

            if (smoothedVelocity > maxLinearVelocity) {
                maxLinearVelocity = smoothedVelocity;
            }

            if (accelWindowTime == 0) {
                accelWindowVelocity = smoothedVelocity;
                accelWindowTime = currentTime;
            } else if (currentTime - accelWindowTime >= ACCEL_WINDOW_S) {
                double linearAcceleration = Math.abs(smoothedVelocity - accelWindowVelocity) / (currentTime - accelWindowTime);
                if (linearAcceleration > maxLinearAcceleration) {
                    maxLinearAcceleration = linearAcceleration;
                }
                accelWindowVelocity = smoothedVelocity;
                accelWindowTime = currentTime;
            }

            double deltaRad = MathFunctions.angleWrapRad(currentPosition.getTheta() - prevPosition.getTheta());
            double angularVelocity = Math.abs(deltaRad / deltaTime); // rad/s

            if (angularVelocity > maxAngularVelocity) {
                maxAngularVelocity = angularVelocity;
            }

            prevPosition = currentPosition;
            prevTime = currentTime;

            double finalMaxLinearAcceleration = maxLinearAcceleration;
            KLog.d("AdaptivePurePursuitRobotData", () -> "maxLinearAcceleration: " + finalMaxLinearAcceleration);
            double finalMaxLinearVelocity = maxLinearVelocity;
            KLog.d("AdaptivePurePursuitRobotData", () -> "maxLinearVelocity: " + finalMaxLinearVelocity);
            double finalMaxAngularVelocity = maxAngularVelocity;
            KLog.d("AdaptivePurePursuitRobotData", () -> "maxAngularVelocity: " + finalMaxAngularVelocity);

        }

        OpModeUtilities.shutdownExecutorService(executorService);

        cleanupRobot();

        double finalMaxLinearAcceleration = maxLinearAcceleration;
        KLog.d("AdaptivePurePursuitRobotData", () -> "maxLinearAcceleration: " + finalMaxLinearAcceleration);
        double finalMaxLinearVelocity = maxLinearVelocity;
        KLog.d("AdaptivePurePursuitRobotData", () -> "maxLinearVelocity: " + finalMaxLinearVelocity);
        double finalMaxAngularVelocity = maxAngularVelocity;
        KLog.d("AdaptivePurePursuitRobotData", () -> "maxAngularVelocity: " + finalMaxAngularVelocity);

    }
}
