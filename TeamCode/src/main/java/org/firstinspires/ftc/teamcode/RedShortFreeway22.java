package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RedShortFreeway22 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, this, telemetry, false, true, true);
        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();
        robot.initVisionProcessing();
        int defaultDelay = 3;
        int maxDelayInSeconds = 3;

        robot.setConfigPresets(defaultDelay, Robot.PARKING_POSITION.BOARD, true);
        robot.buttonConfigAtInit(gamepad1);
        int delay;

        waitForStart();

        while (opModeIsActive()) {

            robot.detectMarkerPosition();
            robot.visionPortal.setProcessorEnabled(robot.markerProcessor, false);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            robot.setMarkerLocation(true, false, robot.markerPos);
            robot.servoToInitPositions();

            robot.shortMoveToBoard2();
            robot.alignToBoardFast(robot.wantedAprTagId);

            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            robot.autoOuttake(true);

            robot.boardToMiddle();

            if (robot.autoDelayInSeconds <= maxDelayInSeconds) {
                delay = robot.autoDelayInSeconds * 1000;
            } else {
                delay = maxDelayInSeconds * 1000;
            }

            robot.middleToStackAndIntake(delay);

            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, true);
            robot.stackToBoard();
            robot.intakeMotor.setPower(0);
            robot.alignToBoardFast(robot.secondWantedTagId);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            robot.trayToOuttakePos(true); // pivot tray to outtake position
            robot.autoOuttake(false);

            robot.configuredParking();

            break;

        }
    }
}