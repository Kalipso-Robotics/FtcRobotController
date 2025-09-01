package com.kalipsorobotics.test.localization;

import android.util.Log;

import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.localization.OdometryFileWriter;
import com.kalipsorobotics.localization.OdometrySensorCombinations;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.GoBildaOdoModule;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Objects;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@TeleOp
public class OdometryEncoderCalcTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);

        DriveAction driveAction = new DriveAction(driveTrain);

        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        GoBildaOdoModule.setInstanceNull();
        GoBildaOdoModule goBildaOdoModule = GoBildaOdoModule.getInstance(opModeUtilities);

        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, goBildaOdoModule);

        OdometryFileWriter odometryFileWriter = new OdometryFileWriter("OdometryTest", opModeUtilities);

        ExecutorService executorService = Executors.newSingleThreadExecutor();
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);
        Position pos1 = SharedData.getOdometryPosition();
        Position pos2 = SharedData.getOdometryPosition();
        double power = 0;
        boolean calibrationComplete = true;
        waitForStart();
        while (opModeIsActive()) {

            /*while (SharedData.getOdometryPositionMap().get(OdometrySensorCombinations.WHEEL_IMU).getCurrentVelocity().getX() <  (50.0/1000.0) && !calibrationComplete) {
                driveTrain.setPower(power);
                power += 0.001;
            }*/
            calibrationComplete = true;
            Log.d("minimumPower", "minimum power " + power);
            driveTrain.setPower(power);


            //goBildaOdoModule.getGoBildaPinpointDriver().update();
            odometryFileWriter.writeOdometryPositionHistory(SharedData.getOdometryPositionMap());
            driveAction.move(gamepad1);


            Log.d("Left_Encoder", "left_encoder " + driveTrain.getLeftEncoder().getCurrentPosition());
            if (driveTrain.getfLeftPower() == 0 || true) {
                Log.d("Odometry_Position_While_Stopped", SharedData.getOdometryPosition() + ", power: fleft " + driveTrain.getfLeftPower() + ", fright " + driveTrain.getfRightPower() + ", bleft " + driveTrain.getbLeftPower() + ", bright " + driveTrain.getbRightPower());
                pos1 = SharedData.getOdometryPosition();
            } else {
                Log.d("Odometry_Position_While_Moving", SharedData.getOdometryPosition() + ", power: fleft " + driveTrain.getfLeftPower() + ", fright " + driveTrain.getfRightPower() + ", bleft " + driveTrain.getbLeftPower() + ", bright " + driveTrain.getbRightPower());
                pos2 = SharedData.getOdometryPosition();
            }
            Log.d("Calibration_Data", "nurture distance " + pos1.distanceTo(pos2) + " nurture angle " + Math.abs(pos1.getTheta() - pos2.getTheta()));
            Log.d("encoders", "count back: " + odometry.getBackEncoderMM() +
                    "  count right: " + odometry.getRightEncoderMM() +
                    "  count left: " + odometry.getLeftEncoderMM());
            Log.d("Velocity", Objects.requireNonNull(SharedData.getOdometryPositionMap().get(OdometrySensorCombinations.WHEEL_IMU)).getCurrentVelocity().toString());
            Log.d("PIN_Position", Objects.requireNonNull(SharedData.getOdometryPositionMap().get(OdometrySensorCombinations.GOBILDA)).getCurrentPosition().toString());
        }
        OpModeUtilities.shutdownExecutorService(executorService);
        odometryFileWriter.close();

    }
}
