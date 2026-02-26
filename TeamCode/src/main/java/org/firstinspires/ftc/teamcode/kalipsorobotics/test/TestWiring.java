package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
public class TestWiring extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor testMotor = hardwareMap.dcMotor.get("testMotor");
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo testServo = hardwareMap.servo.get("testServo");

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine(String.valueOf(testMotor.getCurrentPosition()));
            if (gamepad1.a) {
                testMotor.setPower(0.5);
            } else {
                testMotor.setPower(0);
            }
            if (gamepad1.b) {
                testServo.setPosition(0);
            } else {
                testServo.setPosition(1);
            }

            telemetry.update();
        }
    }
}
