package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class ShortBlueAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, false);
        robot.setUpDrivetrainMotors();
        robot.initVisionProcessing();

        waitForStart();

        while (opModeIsActive()) {
            robot.detectMarkerPosition();
            robot.shortMoveToBoard();
            robot.alignToBoard();
            robot.autoOuttake();
            robot.parkBot(false);
            break;
        }
    }
}
