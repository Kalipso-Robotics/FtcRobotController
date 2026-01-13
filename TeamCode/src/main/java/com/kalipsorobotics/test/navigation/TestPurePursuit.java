package com.kalipsorobotics.test.navigation;

import com.kalipsorobotics.decode.configs.AprilTagConfig;
import com.kalipsorobotics.decode.configs.AprilTagConfigV2;
import com.kalipsorobotics.decode.configs.DrivetrainConfig;
import com.kalipsorobotics.decode.configs.DrivetrainConfigV2;
import com.kalipsorobotics.decode.configs.ModuleConfig;
import com.kalipsorobotics.decode.configs.ModuleConfigV2;
import com.kalipsorobotics.decode.configs.ShooterConfig;
import com.kalipsorobotics.decode.configs.ShooterConfigV2;
import com.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import com.kalipsorobotics.decode.configs.ShooterInterpolationConfigV2;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.localization.OdometryFileWriter;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.GoBildaOdoModule;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.utilities.KOpMode;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@TeleOp
public class TestPurePursuit extends KOpMode {

    @Override
    protected void initializeRobotConfig() {
        super.initializeRobotConfig();
        DrivetrainConfig.BACK_DISTANCE_TO_MID_ROBOT_MM = DrivetrainConfigV2.BACK_DISTANCE_TO_MID_ROBOT_MM;
        DrivetrainConfig.TRACK_WIDTH_MM = DrivetrainConfigV2.TRACK_WIDTH_MM;
        DrivetrainConfig.MM_PER_TICK = DrivetrainConfigV2.MM_PER_TICK;


        AprilTagConfig.APRIL_TAG_HEADING_REL_FIELD_RAD = AprilTagConfigV2.APRIL_TAG_HEADING_REL_FIELD_RAD;
        AprilTagConfig.APRILTAG_X_REL_FIELD_MM = AprilTagConfigV2.APRILTAG_X_REL_FIELD_MM;
        AprilTagConfig.APRILTAG_Y_REL_FIELD_MM = AprilTagConfigV2.APRILTAG_Y_REL_FIELD_MM;
        AprilTagConfig.GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_Z = AprilTagConfigV2.GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_Z;
        AprilTagConfig.GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_X = AprilTagConfigV2.GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_X;
        AprilTagConfig.GOAL_TO_APRIL_TAG_OFFSET_DISTANCE = AprilTagConfigV2.GOAL_TO_APRIL_TAG_OFFSET_DISTANCE;
        AprilTagConfig.TURRET_REL_CAM_POS = AprilTagConfigV2.TURRET_REL_CAM_POS;
        AprilTagConfig.ROBOT_REL_TURRET_POINT = AprilTagConfigV2.ROBOT_REL_TURRET_POINT;

        ModuleConfig.RELEASE_BRAKE_RIGHT_POS = ModuleConfigV2.RELEASE_BRAKE_RIGHT_POS;
        ModuleConfig.RELEASE_BRAKE_LEFT_POS = ModuleConfigV2.RELEASE_BRAKE_LEFT_POS;
        ModuleConfig.STOPPER_SERVO_CLOSED_POS = ModuleConfigV2.STOPPER_SERVO_CLOSED_POS;
        ModuleConfig.STOPPER_SERVO_OPEN_POS = ModuleConfigV2.STOPPER_SERVO_OPEN_POS;

        ShooterConfig.timeToStabilize = ShooterConfigV2.timeToStabilize;
        ShooterConfig.kp = ShooterConfigV2.kp;
        ShooterConfig.ki = ShooterConfigV2.ki;
        ShooterConfig.kd = ShooterConfigV2.kd;
        ShooterConfig.kf = ShooterConfigV2.kf;


        ShooterInterpolationConfig.rpsOffset = ShooterInterpolationConfigV2.rpsOffset;
        ShooterInterpolationConfig.hoodOffset = ShooterInterpolationConfigV2.hoodOffset;
        ShooterInterpolationConfig.HOOD_INIT_OFFSET = ShooterInterpolationConfigV2.HOOD_INIT_OFFSET;
        ShooterInterpolationConfig.MAX_HOOD = ShooterInterpolationConfigV2.MAX_HOOD;
        ShooterInterpolationConfig.MIN_HOOD = ShooterInterpolationConfigV2.MIN_HOOD;
        ShooterInterpolationConfig.FAR_SHOOT_RPS = ShooterInterpolationConfigV2.FAR_SHOOT_RPS;
        ShooterInterpolationConfig.NEAR_SHOOT_RPS = ShooterInterpolationConfigV2.NEAR_SHOOT_RPS;
        ShooterInterpolationConfig.FAR_DISTANCE = ShooterInterpolationConfigV2.FAR_DISTANCE;
        ShooterInterpolationConfig.NEAR_DISTANCE = ShooterInterpolationConfigV2.NEAR_DISTANCE;
        ShooterInterpolationConfig.shooterData = ShooterInterpolationConfigV2.shooterData;


    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.initializeRobot();
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);

        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);

        ExecutorService executorService = Executors.newSingleThreadExecutor();

        PurePursuitAction test = new PurePursuitAction(driveTrain);
//        test.addPoint(0, 0, 0);
//        test.addPoint(1200, 0, 0);
//        test.addPoint(1200, 1200, 90);
//        test.addPoint(600, 1200, 90);

        test.addPoint(1422.4, -508, -90);
        test.addPoint(1422.4, -1219.2, -90);
        test.addPoint(0, 0, 0);



        waitForStart();

        OpModeUtilities.runOdometryExecutorService(executorService, odometry);

        while (opModeIsActive()) {


            test.updateCheckDone();

            if (test.getIsDone()) {
                break;
            }
        }

        OpModeUtilities.shutdownExecutorService(executorService);

    }
}