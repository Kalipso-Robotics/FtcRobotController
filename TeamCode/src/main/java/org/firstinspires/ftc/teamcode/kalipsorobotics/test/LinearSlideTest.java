package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

@TeleOp
public class LinearSlideTest extends LinearOpMode {
    private DcMotor linearSlideMotor;
    private OpModeUtilities opModeUtilities;

    private double d;
    private double i;
    private double p;
    private double f;
    private double s;

    private double integralSum = 0;
    private double lestError = 0;
    private double lastTime = 0;
    private double lastError = 0;

    public void PIDFController(double p, double i, double d, double f){
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }
    public double calculate(double target, double current){
        double currentTime = System.nanoTime() /1E9;
        double deltaTime= currentTime - lastTime;

        if(lastTime == 0) deltaTime = 0;

        double error = target-current;
        double proportionalOut = p * error;
        integralSum += error * deltaTime;
        double integralOut = i*integralSum;
        double derivative = deltaTime > 0 ? (error - lastError)/deltaTime : 0;
        double derivativeOut = d*derivative;
        double feedForwardOut = f;
        double power = proportionalOut + integralOut + derivativeOut + feedForwardOut;
        lastError = error;
        lastTime = currentTime;
        return power;
    }


    @Override
    public void runOpMode() throws InterruptedException {
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        linearSlideMotor = opModeUtilities.getHardwareMap().dcMotor.get("slideMotor");

        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.left_stick_y != 0) {
                linearSlideMotor.setPower(-gamepad1.left_stick_y / 2);
            } else {
                linearSlideMotor.setPower(0);
            }
        }

    }
}
