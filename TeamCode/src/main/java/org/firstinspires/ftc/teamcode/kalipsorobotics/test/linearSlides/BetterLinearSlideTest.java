package org.firstinspires.ftc.teamcode.kalipsorobotics.test.linearSlides;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.kalipsorobotics.PID.PIDFController;

/**
 * Preset height test for the linear slide.
 *
 * <pre>
 *   A  -> bottom   (BOTTOM_TICKS)
 *   X  -> halfway  (halfway between bottom and top)
 *   Y  -> top      (TOP_TICKS)
 *   left stick Y -> jog the target up/down from wherever it is
 * </pre>
 *
 * Gains and the three heights are static so they are live-editable from FTC Dashboard
 * while the op mode is running.
 */
@Config
@TeleOp(name = "Better Linear Slide Test", group = "Test")
public class BetterLinearSlideTest extends LinearOpMode {

    // Change this if the slide is plugged into a differently named port.
    public static final String SLIDE_MOTOR_NAME = "linearSlide1";

    // Travel limits in encoder ticks. TOP_TICKS is the hard stop measured on the old test.
    public static double BOTTOM_TICKS = 0;
    public static double TOP_TICKS = 4660;

    // PIDF gains. kG is the constant power that just holds the slide against gravity.
    public static double KP = 0.015;
    public static double KI = 0.0;
    public static double KD = 0.0001;
    public static double KG = 0.04;

    // How many ticks a full joystick push moves the target per loop.
    public static double JOG_TICKS_PER_LOOP = 40;
    public static double JOYSTICK_DEADBAND = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor slideMotor1 = hardwareMap.dcMotor.get(SLIDE_MOTOR_NAME);

        // If the slide runs the wrong way, flip this to REVERSE.
        slideMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset so the slide's resting position reads 0, then let the PIDF loop own the power
        // while the encoder keeps reporting position.
        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFController slidePID = PIDFController.linearSlide("slideTest", KP, KI, KD, KG)
                .tolerance(15, 20)
                .build();

        double targetPosition = BOTTOM_TICKS;
        String lastPreset = "none";

        telemetry.addLine("A = bottom, X = halfway, Y = top, left stick = jog");
        telemetry.addLine("Make sure the slide is all the way down before pressing start.");
        telemetry.update();

        waitForStart();
        slidePID.reset();

        while (opModeIsActive()) {
            // Pick up any gain edits made from the dashboard mid-run.
            slidePID.setKp(KP);
            slidePID.setKi(KI);
            slidePID.setKd(KD);
            slidePID.setKg(KG);

            double currentPosition = slideMotor1.getCurrentPosition();
            double halfwayTicks = (BOTTOM_TICKS + TOP_TICKS) / 2;

            if (gamepad1.a) {
                targetPosition = BOTTOM_TICKS;
                lastPreset = "bottom";
            } else if (gamepad1.x) {
                targetPosition = halfwayTicks;
                lastPreset = "halfway";
            } else if (gamepad1.y) {
                targetPosition = TOP_TICKS;
                lastPreset = "top";
            }

            // Manual jog: nudges the target so the slide still holds when you let go.
            double joystickInput = -gamepad1.left_stick_y;
            if (Math.abs(joystickInput) > JOYSTICK_DEADBAND) {
                targetPosition += joystickInput * JOG_TICKS_PER_LOOP;
                lastPreset = "manual";
            }

            // Absolute physical constraints.
            if (targetPosition < BOTTOM_TICKS) {
                targetPosition = BOTTOM_TICKS;
            } else if (targetPosition > TOP_TICKS) {
                targetPosition = TOP_TICKS;
            }

            double motorPower = slidePID.calculate(currentPosition, targetPosition);

            // Do not keep driving down once it is sitting on the bottom hard stop.
            if (motorPower < 0 && currentPosition <= BOTTOM_TICKS + 5) {
                motorPower = 0.0;
            }

            slideMotor1.setPower(motorPower);

            telemetry.addData("Preset", lastPreset);
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Current", currentPosition);
            telemetry.addData("Error", slidePID.getError());
            telemetry.addData("Power", motorPower);
            telemetry.addData("At Setpoint", slidePID.atSetpoint());
            telemetry.update();
        }

        slideMotor1.setPower(0);
    }
}
