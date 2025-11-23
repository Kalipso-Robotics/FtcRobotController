package com.kalipsorobotics.test;

import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.KGamePad;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Simple TeleOp")
public class SimpleIntakeTeleop extends LinearOpMode {

    private Intake intake;
    private Stopper stopper;
    private Shooter shooter;
    private OpModeUtilities opModeUtilities;
    private KGamePad kGamePad;

    private static final double SHOOTER_RPS = 40.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize OpModeUtilities
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        // Initialize hardware using modules
        intake = new Intake(opModeUtilities);
        stopper = new Stopper(opModeUtilities);
        shooter = new Shooter(opModeUtilities);

        // Initialize KGamePad wrapper for gamepad1
        kGamePad = new KGamePad(gamepad1);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Right trigger: intake + close stopper");
        telemetry.addData("Instructions", "Left trigger: shooter + open stopper");
        telemetry.update();

        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            boolean rightTriggerPressed = kGamePad.isRightTriggerPressed();
            boolean leftTriggerPressed = kGamePad.isLeftTriggerPressed();

            // Intake control: run if either trigger is pressed
            if (rightTriggerPressed || leftTriggerPressed) {
                intake.getIntakeMotor().setPower(1.0);
            } else {
                // Stop intake only when both triggers are released
                intake.getIntakeMotor().setPower(0.0);
            }

            // Shooter control
            if (leftTriggerPressed) {
                shooter.goToRPS(SHOOTER_RPS);
            } else {
                shooter.stop();
            }

            // Stopper control: only open when left trigger pressed AND shooter at target RPS
            if (leftTriggerPressed && shooter.isAtTargetRPS()) {
                stopper.getStopper().setPosition(stopper.STOPPER_SERVO_OPEN_POS);
            } else {
                // Close stopper in all other cases (collecting, spinning up, or idle)
                stopper.getStopper().setPosition(stopper.STOPPER_SERVO_CLOSED_POS);
            }

            // Telemetry
            telemetry.addData("Right Trigger", rightTriggerPressed ? "PRESSED" : "Released");
            telemetry.addData("Left Trigger", leftTriggerPressed ? "PRESSED" : "Released");
            telemetry.addData("Intake Power", intake.getIntakeMotor().getPower());
            telemetry.addData("Shooter RPS", String.format("%.1f / %.1f", shooter.getRPS(), SHOOTER_RPS));
            telemetry.addData("Shooter Ready", shooter.isAtTargetRPS() ? "YES" : "Spinning up...");
            telemetry.addData("Stopper Position", stopper.getStopper().getPosition());
            telemetry.update();
        }

        // Stop motors when op mode ends
        intake.getIntakeMotor().setPower(0);
        shooter.stop();
    }
}
