package org.firstinspires.ftc.teamcode.kalipsorobotics.test.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Simple drivetrain program for testing 4-motor mecanum/tank drive
 * Controls:
 *   - Left stick Y: Forward/backward
 *   - Left stick X: Strafe left/right (mecanum only)
 *   - Right stick X: Turn left/right
 */

//@TeleOp(name = "Simple Drive Test")
public class SimpleDrive extends LinearOpMode {

    private DcMotor fLeft = null;
    private DcMotor fRight = null;
    private DcMotor bLeft = null;
    private DcMotor bRight = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        telemetry.addData("Status", "Initializing motors...");
        telemetry.update();

        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        // Set motor directions (matching your DriveTrain config)
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior to brake
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "Left stick Y: Forward/Back");
        telemetry.addData("Controls", "Left stick X: Strafe");
        telemetry.addData("Controls", "Right stick X: Turn");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read gamepad inputs
            double drive = -gamepad1.left_stick_y;   // Forward/backward (inverted because Y is reversed)
            double strafe = gamepad1.left_stick_x;    // Strafe left/right
            double turn = gamepad1.right_stick_x;     // Turn left/right

            // Calculate mecanum drive motor powers
            double fLeftPower = drive + strafe + turn;
            double fRightPower = drive - strafe - turn;
            double bLeftPower = drive - strafe + turn;
            double bRightPower = drive + strafe - turn;

            // Normalize powers to keep them within [-1, 1]
            double maxPower = Math.max(Math.abs(fLeftPower),
                             Math.max(Math.abs(fRightPower),
                             Math.max(Math.abs(bLeftPower), Math.abs(bRightPower))));

            if (maxPower > 1.0) {
                fLeftPower /= maxPower;
                fRightPower /= maxPower;
                bLeftPower /= maxPower;
                bRightPower /= maxPower;
            }

            // Set motor powers
            fLeft.setPower(fLeftPower);
            fRight.setPower(fRightPower);
            bLeft.setPower(bLeftPower);
            bRight.setPower(bRightPower);

            // Display telemetry
            telemetry.addData("Drive", "%.2f", drive);
            telemetry.addData("Strafe", "%.2f", strafe);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.addData("", "");
            telemetry.addData("FL Power", "%.2f", fLeftPower);
            telemetry.addData("FR Power", "%.2f", fRightPower);
            telemetry.addData("BL Power", "%.2f", bLeftPower);
            telemetry.addData("BR Power", "%.2f", bRightPower);
            telemetry.update();
        }

        // Stop all motors when opmode ends
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }
}
