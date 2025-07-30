package com.kalipsorobotics.test;

import android.util.Log;

import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.localization.SparkfunOdometry;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.GoBildaPinpointDriver;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        SparkfunOdometry sparkfunOdometry = new SparkfunOdometry(driveTrain, opModeUtilities, 0, 0, Math.toRadians(0));
        WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, 0, 0, Math.toRadians(0));
        DriveAction driveAction = new DriveAction(driveTrain);
        GoBildaPinpointDriver odo  = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        waitForStart();
        while (opModeIsActive()) {

            odo.update();

            driveTrain.setFLeftPower(gamepad1.a ? 0.5 : 0.0);
            driveTrain.setFRightPower(gamepad1.b ? 0.5 : 0.0);
            driveTrain.setBLeftPower(gamepad1.x ? 0.5 : 0.0);
            driveTrain.setBRightPower(gamepad1.y ? 0.5 : 0.0);

            Log.d("encoders", "back wheel " + Math.floor(wheelOdometry.countBack()));
            Log.d("encoders", "left wheel " + Math.floor(wheelOdometry.countLeft()));
            Log.d("encoders", "back wheel Pinpoint " + odo.getEncoderY());
            Log.d("encoders", "left wheel Pinpoint " + odo.getEncoderX());
            Log.d("encoders", "right wheel " + Math.floor(wheelOdometry.countRight()));
            //sleep(1000);

        }
    }
}