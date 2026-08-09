package org.firstinspires.ftc.teamcode.kalipsorobotics.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class HardwareTest extends LinearOpMode {
    private DcMotor bLeft, bRight, fLeft, fRight;
    @Override
    public void runOpMode() throws InterruptedException {
        bLeft = hardwareMap.get(DcMotor.class, "bLeft");
        bRight = hardwareMap.get(DcMotor.class, "bRight");
        fLeft = hardwareMap.get(DcMotor.class, "fLeft");
        fRight = hardwareMap.get(DcMotor.class, "fRight");
        waitForStart();
        while(opModeIsActive()) {
            int encoderPositionLB = bLeft.getCurrentPosition();
            telemetry.addLine("--- Motor Data ---");
            telemetry.addData("Encoder Position", "%d ticks", encoderPositionLB);
            telemetry.addLine();

            int encoderPositionRB = bRight.getCurrentPosition();
            telemetry.addLine("--- Motor Data ---");
            telemetry.addData("Encoder Position", "%d ticks", encoderPositionRB);
            telemetry.addLine();

            int encoderPositionLF = fLeft.getCurrentPosition();
            telemetry.addLine("--- Motor Data ---");
            telemetry.addData("Encoder Position", "%d ticks", encoderPositionLF);
            telemetry.addLine();

            int encoderPositionRF = fRight.getCurrentPosition();
            telemetry.addLine("--- Motor Data ---");
            telemetry.addData("Encoder Position", "%d ticks", encoderPositionRF);
            telemetry.addLine();
        }
    }
}
