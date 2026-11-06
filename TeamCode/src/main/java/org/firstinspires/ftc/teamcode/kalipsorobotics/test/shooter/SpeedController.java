package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KMotor;


@TeleOp
public class SpeedController extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor cheese = hardwareMap.dcMotor.get("motor0");
        KMotor motor0 = new KMotor(cheese, 0, 0, 0, 0);
        DcMotor motor1 = hardwareMap.dcMotor.get("motor1");
        DcMotor motor2 = hardwareMap.dcMotor.get("motor2");

//        Servo servo0 = hardwareMap.servo.get("servo0");

        double position = 0.5;

        double power0 = 0;

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.left_stick_y < 0) {
                power0 -= 0.001;
                motor0.setPower(power0);

            } else if (gamepad1.left_stick_y > 0) {
                power0 += 0.001;
                motor0.setPower(power0);

            } else if (gamepad1.a){
                motor0.setPower(0);
                power0 = 0;
            }

            double power = gamepad1.right_trigger;

            motor1.setPower(power);
            motor2.setPower(-power);


//            if (gamepad1.dpad_up) {
//                position += 0.0005;
//                servo0.setPosition(position);
//            } else if (gamepad1.dpad_down) {
//                position -= 0.0005;
//                servo0.setPosition(position);
//            }

            telemetry.addData("Power 0: ", power0); //-0.47
            telemetry.addData("Power 1: ", power);
            telemetry.addData("Power 2: ", -power);
            telemetry.addData("rpm", motor0.getRPS());
            telemetry.addData("position: ", position);
            telemetry.update();



        }
    }


}
