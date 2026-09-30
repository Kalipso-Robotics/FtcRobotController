package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class SpeedController extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor motor0 = hardwareMap.dcMotor.get("motor0");

        Servo servo0 = hardwareMap.servo.get("servo0");

        double position = 0.5;

        double power = 0;

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.left_stick_y != 0) {
                power -= 0.001;
                motor0.setPower(power);

            } else if (gamepad1.a){
                motor0.setPower(0);
            }

            if (gamepad1.dpad_up) {
                position += 0.0005;
                servo0.setPosition(position);
            } else if (gamepad1.dpad_down) {
                position -= 0.0005;
                servo0.setPosition(position);
            }

            telemetry.addData("Power: ", power);
            telemetry.addData("position: ", position);
            telemetry.update();



        }
    }


}
