package com.kalipsorobotics.test.checkStuck;

import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.actions.CheckStuckRobot;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.localization.SparkfunOdometry;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.Path;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestCheckStuck extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, new Position(0, 0, 0));
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

            if (checkStuck.isStuck(currentPos, xDelta, yDelta, thetaDelta)) {
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
