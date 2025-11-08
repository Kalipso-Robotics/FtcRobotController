package com.kalipsorobotics.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
public class IntakeMotorTest extends LinearOpMode {
    boolean toggle;
    double increment = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                toggle = true;
            }
            else if (gamepad1.b) {
                toggle = false;
            }

            if (gamepad1.x) {
                increment = Math.max(increment-0.1,-1);
            }
            else if (gamepad1.y) {
                increment = Math.min(increment+0.1,1);
            }
            if (toggle){
                intakeMotor.setPower(increment);
            }
            else {
                intakeMotor.setPower(0);
            }

        }

    }
}
