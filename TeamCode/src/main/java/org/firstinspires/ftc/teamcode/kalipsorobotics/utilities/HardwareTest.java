package org.firstinspires.ftc.teamcode.kalipsorobotics.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.CalculateTickPer;

import java.time.chrono.ThaiBuddhistEra;

@TeleOp
public class HardwareTest extends LinearOpMode {
    private DcMotor leftBack, rightBack, leftFront, rightFront;
    @Override
    public void runOpMode() throws InterruptedException {
        leftBack = hardwareMap.get(DcMotor.class, "bLeft");
        rightBack = hardwareMap.get(DcMotor.class, "bRight");
        leftFront = hardwareMap.get(DcMotor.class, "fLeft");
        rightFront = hardwareMap.get(DcMotor.class, "fRight");
        waitForStart();
        while(opModeIsActive()) {
            int encoderPositionLB = leftBack.getCurrentPosition();
            telemetry.addLine("--- Motor Data ---");
            telemetry.addData("Encoder Position", "%d ticks", encoderPositionLB);
            telemetry.addLine();

            int encoderPositionRB = rightBack.getCurrentPosition();
            telemetry.addLine("--- Motor Data ---");
            telemetry.addData("Encoder Position", "%d ticks", encoderPositionRB);
            telemetry.addLine();

            int encoderPositionLF = leftFront.getCurrentPosition();
            telemetry.addLine("--- Motor Data ---");
            telemetry.addData("Encoder Position", "%d ticks", encoderPositionLF);
            telemetry.addLine();

            int encoderPositionRF = rightFront.getCurrentPosition();
            telemetry.addLine("--- Motor Data ---");
            telemetry.addData("Encoder Position", "%d ticks", encoderPositionRF);
            telemetry.addLine();
        }
    }
}
