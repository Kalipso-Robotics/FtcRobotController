package com.kalipsorobotics.actions.checkStuck.checkStuck;

import android.util.Log;

import com.kalipsorobotics.actions.checkStuck.CheckStuckRobot;
import com.kalipsorobotics.modules.GoBildaOdoModule;
import com.kalipsorobotics.modules.GoBildaPinpointDriver;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.localization.SparkfunOdometry;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestCheckStuck extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        GoBildaOdoModule goBildaOdoModule =  GoBildaOdoModule.getInstance(opModeUtilities);
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        Odometry wheelOdometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, goBildaOdoModule, new Position(0, 0, 0));
        PurePursuitAction purePursuitAction = new PurePursuitAction(driveTrain, wheelOdometry, 0, 0);
        DriveAction driveAction = new DriveAction(driveTrain);
        CheckStuckRobot checkStuck = new CheckStuckRobot(driveTrain, wheelOdometry, opModeUtilities, purePursuitAction);
        SparkfunOdometry sparkfunOdometry = new SparkfunOdometry(driveTrain, opModeUtilities, 0, 0, 0);
        waitForStart();
        while (opModeIsActive()) {
            sparkfunOdometry.updatePosition();
            Position currentPos = wheelOdometry.updateDefaultPosition();

            // Compute deltas ONCE per loop
            double xDelta = checkStuck.getXDelta(currentPos);
            double yDelta = checkStuck.getYDelta(currentPos);
            double thetaDelta = checkStuck.getThetaDelta(currentPos);

            if (checkStuck.isStuck(currentPos)) {
                telemetry.addLine("robot is stuck");
            } else {
                telemetry.addLine("robot is fine");
            }

            Log.d("check stuks", "x delta: " + xDelta + " y delta: " + yDelta + " theta delta: " + thetaDelta);
            driveAction.move(gamepad1);
            telemetry.update();
        }

    }
}
