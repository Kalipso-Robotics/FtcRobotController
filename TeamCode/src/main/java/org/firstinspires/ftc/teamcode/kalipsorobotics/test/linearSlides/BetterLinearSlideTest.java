package org.firstinspires.ftc.teamcode.kalipsorobotics.test.linearSlides;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class BetterLinearSlideTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        DcMotor slideMotor1 = hardwareMap.dcMotor.get("PLACEHOLDER");
        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {

            if(gamepad1.a){
//                PLACEHOLDER.setTargetPosition(500);
            }

        }
    }
}
