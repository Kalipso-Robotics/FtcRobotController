package org.firstinspires.ftc.teamcode.kalipsorobotics.test.linearSlides;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;

/**
 * Joystick-driven power test for up to 2 slide motors.
 * Right/left bumper tunes the shared power ceiling down/up from the default [-1, 1] range.
 */
@TeleOp
public class BetterLinearSlideTest extends LinearOpMode {

    private static final double POWER_STEP = 0.1;
    private static final double MIN_MAX_POWER = 0.1;
    private static final double MAX_MAX_POWER = 1.0;

    private double maxPower = MAX_MAX_POWER;
    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor slideMotor1 = hardwareMap.dcMotor.get("PLACEHOLDER1");
        slideMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor slideMotor2 = hardwareMap.dcMotor.get("PLACEHOLDER2");
        slideMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            maxPower = updateMaxPower(maxPower);

            driveSlide(slideMotor1, -gamepad1.left_stick_y, maxPower);
            driveSlide(slideMotor2, -gamepad1.right_stick_y, maxPower);

            telemetry.addData("maxPower", maxPower);
            telemetry.addData("motor1 power", slideMotor1.getPower());
            telemetry.addData("motor2 power", slideMotor2.getPower());
            telemetry.update();
        }
    }

    private double updateMaxPower(double currentMaxPower) {
        double newMaxPower = currentMaxPower;
        if (gamepad1.right_bumper && !prevRightBumper) {
            newMaxPower += POWER_STEP;
        }
        if (gamepad1.left_bumper && !prevLeftBumper) {
            newMaxPower -= POWER_STEP;
        }
        prevRightBumper = gamepad1.right_bumper;
        prevLeftBumper = gamepad1.left_bumper;
        return MathFunctions.clamp(newMaxPower, MIN_MAX_POWER, MAX_MAX_POWER);
    }

    private void driveSlide(DcMotor motor, double stickInput, double maxPower) {
        motor.setPower(MathFunctions.clamp(stickInput, -maxPower, maxPower));
    }
}
