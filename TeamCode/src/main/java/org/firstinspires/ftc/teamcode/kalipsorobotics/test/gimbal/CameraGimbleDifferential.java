package org.firstinspires.ftc.teamcode.kalipsorobotics.test.gimbal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class CameraGimbleDifferential extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Servo servo0 = hardwareMap.servo.get("servo0");
        double position0 = 0.5;
        Servo servo1 = hardwareMap.servo.get("servo1");
        double position1 = 0.5;

        servo0.setPosition(position0);
        servo1.setPosition(position1);

        waitForStart();
        while (opModeIsActive()) {

            if(gamepad1.left_stick_x < 0) {
                position0 += 0.0005;
                servo0.setPosition(position0);

                position1 -= 0.0005;
                servo1.setPosition(position1);

            } else if (gamepad1.left_stick_x > 0) {
                position0 -= 0.0005;
                servo0.setPosition(position0);

                position1 += 0.0005;
                servo1.setPosition(position1);
            }

            if(gamepad1.left_stick_y > 0) {
                position1 += 0.005;
                servo1.setPosition(position1);

                position0 += 0.005;
                servo0.setPosition(position0);

            } else if (gamepad1.left_stick_y < 0) {
                position1 -= 0.005;
                servo1.setPosition(position1);

                position0 -= 0.005;
                servo0.setPosition(position0);
            }

            if (position1 > 1) {
                position1 = 1;
            }
            if (position0 > 1) {
                position0 = 1;
            }

            if (position1 < 0) {
                position1 = 0;
            }
            if (position0 < 0) {
                position0 = 0;
            }

            telemetry.addData("Position 1", position1);
            telemetry.addData("Position 0", position0);
            telemetry.update();


        }

    }
}
