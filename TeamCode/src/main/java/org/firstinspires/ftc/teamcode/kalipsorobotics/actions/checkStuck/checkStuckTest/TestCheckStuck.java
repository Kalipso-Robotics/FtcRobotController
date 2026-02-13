package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.checkStuck.checkStuckTest;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.checkStuck.CheckStuckRobot;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.GoBildaOdoModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.DriveAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
public class TestCheckStuck extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        GoBildaOdoModule goBildaOdoModule =  GoBildaOdoModule.getInstance(opModeUtilities);
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, new Position(0, 0, 0));
        PurePursuitAction purePursuitAction = new PurePursuitAction(driveTrain, 0.0, 0.0);
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
