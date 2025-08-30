// PurePursuitFollowerLinearOpMode.java
// LinearOpMode that uses the Kalipso Robotics odometry stack and a TFLite inverse model
// to follow a path with Pure Pursuit.
//
// Place the assets in: TeamCode/src/main/assets/
//   - mecanum_inverse.tflite
//   - mecanum_inverse_norm.json
// Add dependency in TeamCode/build.gradle:
//   implementation "org.tensorflow:tensorflow-lite:2.13.0"
//
// If package names differ, adjust the `package` line and imports accordingly.

package com.kalipsorobotics.test.navigation;

import android.util.Log;

import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.localization.OdometryFileWriter;
import com.kalipsorobotics.localization.OdometrySensorCombinations;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.PositionHistory;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.GoBildaOdoModule;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.navigation.PurePursuitTFLite;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.json.JSONException;

import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@TeleOp(name="TestNavigation", group="Examples")
public class TestNavigation extends LinearOpMode {

    private PurePursuitTFLite pp;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Your existing stack wiring (from the example) ---
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);


        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        GoBildaOdoModule.setInstanceNull();
        GoBildaOdoModule goBildaOdoModule = GoBildaOdoModule.getInstance(opModeUtilities);

        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, goBildaOdoModule);


        ExecutorService executorService = Executors.newSingleThreadExecutor();
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);



        // --- Load inverse model ---
        try {
            pp = PurePursuitTFLite.fromAssets(
                    hardwareMap.appContext,
                    "mecanum_inverse.tflite",
                    "mecanum_inverse_norm.json"
            );
        } catch (IOException | JSONException e) {
            telemetry.addData("ERROR", "Model load failed: %s", e.getMessage());
            telemetry.update();
        }

        // --- Build a sample path (mm, mm, rad). Replace with your waypoints. ---
        List<PurePursuitTFLite.Pose2d> path = Arrays.asList(
                new PurePursuitTFLite.Pose2d(0,   0,   Math.toRadians(0)),
                new PurePursuitTFLite.Pose2d(600, 0, Math.toRadians(-90)),
                new PurePursuitTFLite.Pose2d(600, -600, Math.toRadians(-90)),
                new PurePursuitTFLite.Pose2d(600, 1200, Math.toRadians(-90))
        );
        if (pp != null) {
            pp.setPath(path);
        }

        telemetry.addLine("Initialized. Press START to begin Pure Pursuit.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            PoseMMRad curPose = readPoseFromSharedData();

            if (pp == null) {
                telemetry.addLine("PurePursuitTFLite not initialized.");
                telemetry.update();
                continue;
            }

            //TODO To Be Implemented

            telemetry.addData("Pose (mm, rad)", "(%.1f, %.1f, %.3f)", curPose.x_mm, curPose.y_mm, curPose.heading_rad);
            telemetry.addData("Finished", pp.isFinished());
            telemetry.update();


        }

    }


    // --- Data container for pose ---
    private static class PoseMMRad {
        final double x_mm, y_mm, heading_rad;
        PoseMMRad(double x_mm, double y_mm, double heading_rad) {
            this.x_mm = x_mm; this.y_mm = y_mm; this.heading_rad = heading_rad;
        }
    }

    // --- Pull pose (x_mm, y_mm, heading_rad) from the Kalipso SharedData map using WHEEL_IMU ---
    // This uses mild reflection so it will work even if your Position class method names vary.
    private PoseMMRad readPoseFromSharedData() {
        Position position = SharedData.getOdometryPosition();
        return new PoseMMRad(position.getX(), position.getY(), position.getTheta());

    }
}
