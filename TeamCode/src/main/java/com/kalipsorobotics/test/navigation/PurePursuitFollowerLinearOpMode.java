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

@TeleOp(name="Pure Pursuit Linear (TFLite Inverse)", group="Examples")
public class PurePursuitFollowerLinearOpMode extends LinearOpMode {

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

        //OdometryFileWriter odometryFileWriter = new OdometryFileWriter("PurePursuitRun", opModeUtilities);

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

            // Optional logging, and ensures odometry keeps writing a trace if desired
            //odometryFileWriter.writeOdometryPositionHistory(SharedData.getOdometryPositionMap());

            // --- Read current pose from your SharedData odometry ---
            PoseMMRad curPose = readPoseFromSharedData();

            if (pp == null) {
                telemetry.addLine("PurePursuitTFLite not initialized.");
                telemetry.update();
                continue;
            }

            PurePursuitTFLite.MotorPowers mp = pp.update(new PurePursuitTFLite.Pose2d(curPose.x_mm, curPose.y_mm, curPose.heading_rad));

            driveTrain.setPower(mp.fL, mp.fR, mp.bL, mp.bR);

            Log.d("MotorPowers", String.format("fL=%.3f fR=%.3f bL=%.3f bR=%.3f", mp.fL, mp.fR, mp.bL, mp.bR));
            telemetry.addData("Pose (mm, rad)", "(%.1f, %.1f, %.3f)", curPose.x_mm, curPose.y_mm, curPose.heading_rad);
            telemetry.addData("Finished", pp.isFinished());
            telemetry.update();

            if (pp.isFinished()) {
                stopAllMotors(driveTrain);
                break;
            }
        }

        stopAllMotors(driveTrain);
        //odometryFileWriter.close();
    }

    private void stopAllMotors(DriveTrain driveTrain) {
        driveTrain.setPower(0);
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

    // --- Reflection helpers (best-effort without tight coupling to your Position API) ---
    private static Object invokeFirst(Object obj, String[] methods) {
        for (String m : methods) {
            try {
                Method method = obj.getClass().getMethod(m);
                method.setAccessible(true);
                return method.invoke(obj);
            } catch (Exception ignored) { }
        }
        return null;
    }

    private static double tryNumber(Object obj, String[] methodNames, String[] fieldNames) throws Exception {
        // Try methods first
        for (String m : methodNames) {
            try {
                Method method = obj.getClass().getMethod(m);
                method.setAccessible(true);
                Object val = method.invoke(obj);
                if (val instanceof Number) return ((Number) val).doubleValue();
            } catch (Exception ignored) { }
        }
        // Then try fields
        for (String f : fieldNames) {
            try {
                Field field = obj.getClass().getField(f);
                field.setAccessible(true);
                Object val = field.get(obj);
                if (val instanceof Number) return ((Number) val).doubleValue();
            } catch (Exception ignored) { }
        }
        throw new IllegalStateException("Could not extract numeric value from " + obj.getClass().getName());
    }
}
