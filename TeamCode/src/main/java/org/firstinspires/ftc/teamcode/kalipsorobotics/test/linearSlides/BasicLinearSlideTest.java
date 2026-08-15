package org.firstinspires.ftc.teamcode.kalipsorobotics.test.linearSlides;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class BasicLinearSlideTest extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {


        DcMotor slideMotor1 = hardwareMap.dcMotor.get("motor0");



        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.left_stick_y != 0) {

                slideMotor1.setPower(-gamepad1.left_stick_y * 1);
            } else {
                slideMotor1.setPower(0);
            }


            telemetry.addData("position: ", slideMotor1.getCurrentPosition());

        }


    }


}
