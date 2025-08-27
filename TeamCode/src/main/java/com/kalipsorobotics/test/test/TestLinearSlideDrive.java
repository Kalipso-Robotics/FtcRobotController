package com.kalipsorobotics.test.test;

import android.util.Log;

import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.localization.OdometryFileWriter;
import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.GoBildaOdoModule;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@TeleOp
public class TestLinearSlideDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        waitForStart();

        DcMotor fLeft = hardwareMap.dcMotor.get("fLeft");
        DcMotor fRight = hardwareMap.dcMotor.get("fRight");
        DcMotor bLeft = hardwareMap.dcMotor.get("bLeft");
        DcMotor bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {

            double forward = 0.5 * -gamepad1.left_stick_y * -gamepad1.left_stick_y * -gamepad1.left_stick_y; //cube so slower is slower and faster is faster
            double turn = 0.5 * gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x;
            double strafe = 0.5 * gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x;

            double fLeftPower = (forward + strafe + turn);
            double fRightPower = (forward - strafe - turn);
            double bLeftPower = (forward - strafe + turn);
            double bRightPower = (forward + strafe - turn);

            double absMaxPower = MathFunctions.maxAbsValueDouble(fLeftPower, fRightPower, bLeftPower, bRightPower);
            if (absMaxPower > 1) {
                fLeftPower = fLeftPower / absMaxPower;
                fRightPower = fRightPower / absMaxPower;
                bLeftPower = bLeftPower / absMaxPower;
                bRightPower = bRightPower / absMaxPower;
            }

            fLeft.setPower(fLeftPower);
            fRight.setPower(fRightPower);
            bLeft.setPower(bLeftPower);
            bRight.setPower(bRightPower);

        }

    }
}
