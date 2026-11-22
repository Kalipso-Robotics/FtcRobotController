package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KMotor;


@Config
@TeleOp
public class SpeedController extends LinearOpMode {

    private static final double SHOOTER_POWER_STEP = 0.001;
    private static final double SERVO_STEP = 0.0005;

    private static final double STOPPER_POSITION_A = 0.15;
    private static final double STOPPER_POSITION_Y = 0.3;

    // B = push sequence: run intake (stopper closed) for PRE_OPEN_MS, open stopper and keep intaking
    // for SHOOT_MS, then close stopper and stop intake. Both times tunable on the right stick or FTC Dashboard.
    private static final double STOPPER_OPEN_POS = 0.362;   // same as Y
    private static final double STOPPER_CLOSED_POS = 0.449; // same as A
    public static double PRE_OPEN_MS = 300; // intake runs this long before the stopper opens
    public static double SHOOT_MS = 400;    // stopper open + intake running, then close
    private static final double TIME_STEP_MS = 2; // per loop while the right stick is held

    private enum PushState { IDLE, PRE_OPEN, SHOOTING }

    @Override
    public void runOpMode() throws InterruptedException {

        // ---- Motors (names are the hardware config names) ----
        KMotor shooterMotor = new KMotor(hardwareMap.dcMotor.get("shooter"), 0, 0, 0, 0);
        DcMotor intakeMotor1 = hardwareMap.dcMotor.get("intakeFront");
        DcMotor intakeMotor2 = hardwareMap.dcMotor.get("intakeBack");

        // ---- Servos (names are the hardware config names) ----
        Servo hoodServo = hardwareMap.servo.get("hood"); // 0.929
        Servo stopperServo = hardwareMap.servo.get("stopper");
        Servo turret1Servo = hardwareMap.servo.get("turret1"); //0.84
        Servo turret2Servo = hardwareMap.servo.get("turret2");

        // ---- Targets ----
        double shooterPower = 0;
        double stopperPosition = 0.5;
        double hoodPosition = 0.96;
        double turret1Position = 0.233;
        double turret2Position = 0.5;

        double intakePower = 1;

        PushState pushState = PushState.IDLE;
        ElapsedTime pushTimer = new ElapsedTime();
        boolean prevB = false;

        waitForStart();
        while (opModeIsActive()) {

            // Shooter (motor0): left stick Y adjusts power, X stops
            if (gamepad1.left_stick_y < 0) {
                shooterPower -= SHOOTER_POWER_STEP;
                shooterMotor.setPower(shooterPower);
            } else if (gamepad1.left_stick_y > 0) {
                shooterPower += SHOOTER_POWER_STEP;
                shooterMotor.setPower(shooterPower);
            } else if (gamepad1.x) {
                shooterPower = 0;
                shooterMotor.setPower(shooterPower);
            }

            // Intake (motor1 + motor2, opposite directions): right trigger
            if (gamepad1.right_trigger != 0) {
                intakePower += SERVO_STEP;
            } else if (gamepad1.left_trigger != 0) {
                intakePower -= SERVO_STEP;
            }

            // Push times: right stick up/down = pre-open time, right/left = shoot time
            if (gamepad1.right_stick_y < -0.5) {
                PRE_OPEN_MS += TIME_STEP_MS;
            } else if (gamepad1.right_stick_y > 0.5) {
                PRE_OPEN_MS = Math.max(0, PRE_OPEN_MS - TIME_STEP_MS);
            }
            if (gamepad1.right_stick_x > 0.5) {
                SHOOT_MS += TIME_STEP_MS;
            } else if (gamepad1.right_stick_x < -0.5) {
                SHOOT_MS = Math.max(0, SHOOT_MS - TIME_STEP_MS);
            }

            // B (first press): push sequence. Ignored while one is already running.
            boolean bPressed = gamepad1.b && !prevB;
            prevB = gamepad1.b;
            if (bPressed && pushState == PushState.IDLE) {
                stopperServo.setPosition(STOPPER_CLOSED_POS);
                pushTimer.reset();
                pushState = PushState.PRE_OPEN;
            }
            if (pushState == PushState.PRE_OPEN && pushTimer.milliseconds() >= PRE_OPEN_MS) {
                stopperServo.setPosition(STOPPER_OPEN_POS);
                pushTimer.reset();
                pushState = PushState.SHOOTING;
            }
            if (pushState == PushState.SHOOTING && pushTimer.milliseconds() >= SHOOT_MS) {
                stopperServo.setPosition(STOPPER_CLOSED_POS);
                pushState = PushState.IDLE;
            }

            if (pushState != PushState.IDLE) {
                intakeMotor1.setPower(intakePower);
                intakeMotor2.setPower(-intakePower);
            } else if (gamepad1.right_bumper) {
                stopperServo.setPosition(0.449);
                intakeMotor1.setPower(intakePower);
                intakeMotor2.setPower(-intakePower);
            } else if (gamepad1.left_bumper) {
                intakeMotor1.setPower(-intakePower);
                intakeMotor2.setPower(intakePower);
            } else {
                intakeMotor1.setPower(0);
                intakeMotor2.setPower(0);
            }

            // Hood (servo0): dpad up / down
            if (gamepad1.dpad_up) {
                hoodPosition += SERVO_STEP;
                hoodServo.setPosition(hoodPosition);
            } else if (gamepad1.dpad_down) {
                hoodPosition -= SERVO_STEP;
                hoodServo.setPosition(hoodPosition);
            }

            // Stopper (servo1): A / Y jump to fixed positions
            if (pushState != PushState.IDLE) {
                // PushBall sequence owns the stopper
            } else if (gamepad1.a) {
                stopperPosition += SERVO_STEP;
                stopperServo.setPosition(0.449); //0.449
            } else if (gamepad1.y) {
                stopperPosition -= SERVO_STEP;
                stopperServo.setPosition(0.362); //0.362
            }

            // Turret 1 (servo2): dpad right / left
            if (gamepad1.dpad_right) {
                turret1Position = Math.min(1, turret1Position + SERVO_STEP);
                turret1Servo.setPosition(turret1Position);
            } else if (gamepad1.dpad_left) {
                turret1Position = Math.max(0, turret1Position - SERVO_STEP);
                turret1Servo.setPosition(turret1Position);
            }


            telemetry.addLine("--- Controls (gamepad1) ---");
            telemetry.addLine("Shooter (motor0): left stick Y (up = -, down = +), X = stop");
            telemetry.addLine("Intake: RB = in, LB = out, RT / LT = power up / down");
            telemetry.addLine("Hood (servo0): dpad up / down");
            telemetry.addLine("Stopper (servo1): A = close " + STOPPER_CLOSED_POS + ", Y = open " + STOPPER_OPEN_POS);
            telemetry.addLine("Turret 1 (servo2): dpad right / left");
            telemetry.addLine("Turret 2 (servo3): right bumper / left bumper");
            telemetry.addLine("B = push (intake, open stopper, close). Right stick Y = pre-open time, X = shoot time");
            telemetry.addData("Push state", pushState);
            telemetry.addData("Push pre-open / shoot (ms)", "%.0f / %.0f", PRE_OPEN_MS, SHOOT_MS);

            telemetry.addLine("--- Motors ---");
            telemetry.addData("Shooter power (motor0)", "%.3f", shooterPower); // -0.47
            telemetry.addData("Shooter RPS (motor0)", "%.2f", shooterMotor.getRPS());
            telemetry.addData("Intake 1 power (motor1)", "%.2f", intakePower);
            telemetry.addData("Intake 2 power (motor2)", "%.2f", -intakePower);

            telemetry.addLine("--- Servos (target / actual) ---");
            telemetry.addData("Hood (servo0)", "%.4f / %.4f", hoodPosition, hoodServo.getPosition());
            telemetry.addData("Stopper (servo1)", "%.4f", stopperServo.getPosition());
            telemetry.addData("Turret 1 (servo2)", "%.4f / %.4f", turret1Position, turret1Servo.getPosition());
            telemetry.addData("Turret 2 (servo3)", "%.4f / %.4f", turret2Position, turret2Servo.getPosition());
            telemetry.update();
        }
    }
}
