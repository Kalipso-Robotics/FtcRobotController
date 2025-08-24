package com.kalipsorobotics.test.localization;

import android.util.Log;

import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.localization.OdometryFileWriter;
import com.kalipsorobotics.localization.OdometrySensorCombinations;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.GoBildaOdoModule;
import com.kalipsorobotics.modules.GoBildaPinpointDriver;
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
        waitForStart();

        while (opModeIsActive()) {

            //goBildaOdoModule.getGoBildaPinpointDriver().update();
            odometryFileWriter.writeOdometryPositionHistory(SharedData.getOdometryPositionMap());
            driveAction.move(gamepad1);



            Log.d("Odometry_Position", SharedData.getOdometryPosition().toString());
            Log.d("encoders", "count back: " + odometry.getBackEncoderMM() +
                    "  count right: " + odometry.getRightEncoderMM() +
                    "  count left: " + odometry.getLeftEncoderMM());
            Log.d("Velocity", odometry.getCurrentPositionHistory().getCurrentVelocity().toString());
            Log.d("PIN_Position", Objects.requireNonNull(SharedData.getOdometryPositionMap().get(OdometrySensorCombinations.GOBILDA)).getCurrentPosition().toString());
        }
        odometryFileWriter.close();

        OpModeUtilities.shutdownExecutorService(executorService);
    }
}
