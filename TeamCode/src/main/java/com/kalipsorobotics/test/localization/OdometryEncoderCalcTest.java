package com.kalipsorobotics.test.localization;

import android.util.Log;

import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.GoBildaOdoModule;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

        GoBildaOdoModule goBildaOdoModule = GoBildaOdoModule.getInstance(opModeUtilities);

        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, goBildaOdoModule);



        waitForStart();
        while (opModeIsActive()) {

            driveAction.move(gamepad1);

            Log.d("Position", odometry.updateDefaultPosition().toString());
            Log.d("encoders", "count back: " + odometry.countBack() + "  count right: " + odometry.countRight() + "  count left: " + odometry.countLeft());
            Log.d("Velocity", odometry.getCurrentPositionHistory().getCurrentVelocity().toString());



        }

    }

}
