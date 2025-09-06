package com.kalipsorobotics.actions.checkStuck.checkStuckTest;

import android.util.Log;

import com.kalipsorobotics.actions.checkStuck.CheckStuckRobot;
import com.kalipsorobotics.modules.GoBildaOdoModule;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
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
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, goBildaOdoModule, new Position(0, 0, 0));
        PurePursuitAction purePursuitAction = new PurePursuitAction(driveTrain, 0, 0);
        DriveAction driveAction = new DriveAction(driveTrain);
        CheckStuckRobot checkStuck = new CheckStuckRobot(driveTrain, odometry, opModeUtilities, purePursuitAction);
        waitForStart();
        while (opModeIsActive()) {
            Position currentPos = odometry.update();


            if (checkStuck.isStuck(currentPos)) {
                telemetry.addLine("robot is stuck");
            } else {
                telemetry.addLine("robot is fine");
            }

            driveAction.move(gamepad1);
            telemetry.update();
        }

    }
}
