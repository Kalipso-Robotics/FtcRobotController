package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.configs.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.ResetOdometryToPos;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KMotor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;


@Config
@TeleOp
public class SpeedController extends KOpMode {

    private static final String TAG = "SpeedController";

    private static final double SHOOTER_POWER_STEP = 0.001;
    private static final double SERVO_STEP = 0.0005;

    private static final double STOPPER_POSITION_A = 0.15;
    private static final double STOPPER_POSITION_Y = 0.3;

    // B = push sequence: run intake (stopper closed) for PRE_OPEN_MS, open stopper and keep intaking
    // for SHOOT_MS, then close stopper and stop intake. Both times tunable on the right stick or FTC Dashboard.
    // Stopper open/closed defaults; adjust live on gamepad2 dpad and copy the telemetry values into ModuleConfig by hand.
    private static final double STOPPER_OPEN_POS_DEFAULT = 0.362;
    private static final double STOPPER_CLOSED_POS_DEFAULT = 0.449;
    public static double PRE_OPEN_MS = 300; // intake runs this long before the stopper opens
    public static double SHOOT_MS = 400;    // stopper open + intake running, then close
    private static final double TIME_STEP_MS = 2; // per loop while the right stick is held

    private enum PushState { IDLE, PRE_OPEN, SHOOTING }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        OpModeUtilities.startOdometry(opModeUtilities, odoExecutorService, driveTrain, imuModule, null); // keeps the pose from auto, else the corner

        // Turret auto-align (gamepad2 A toggles). Starts off so the dpad drives the servo raw.
        // Note: Turret sets turret1's PWM range to 500-2500us, so raw positions match TurretConfig.SERVO_ZERO_POS.
        Turret turret = Turret.getInstance(opModeUtilities);
        turretAutoAlignTeleOp = new TurretAutoAlign(opModeUtilities, turret); // aims at SharedData.getTargetPoint(); updateActions() ticks it
        turretAutoAlignTeleOp.setTurretRunMode(TurretRunMode.STOP);
        boolean autoAlign = false;


        // ---- Motors (names are the hardware config names) ----
        KMotor shooterMotor = new KMotor(hardwareMap.dcMotor.get("shooter"), 0, 0, 0, 0);
        DcMotor intakeMotor1 = hardwareMap.dcMotor.get("intakeFront");
        DcMotor intakeMotor2 = hardwareMap.dcMotor.get("intakeBack");

        // ---- Servos (names are the hardware config names) ----
        Servo hoodServo = hardwareMap.servo.get("hood"); // 0.929
        Servo stopperServo = hardwareMap.servo.get("stopper");
        Servo turret1Servo = hardwareMap.servo.get("turret1"); //0.84
        // Control hub: 0 = intakePivot1, 1 = turret1, 2 = intakePivot2
        Servo intakePivot1Servo = hardwareMap.servo.get("intakePivot1");
        Servo intakePivot2Servo = hardwareMap.servo.get("intakePivot2");


        // ---- Targets ----
        double shooterPower = 0;
        double stopperPosition = 0.5;
        double hoodPosition = 0.96;
        double turret1Position = 0.233;
        double intakePivot1Position = 0.5;
        double intakePivot2Position = 0.5;

        double intakePower = 1;
        double stopperOpenPos = STOPPER_OPEN_POS_DEFAULT;
        double stopperClosedPos = STOPPER_CLOSED_POS_DEFAULT;

        PushState pushState = PushState.IDLE;
        ElapsedTime pushTimer = new ElapsedTime();
        boolean prevB = false;

        waitForStart();
        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache(); // initializeRobot() set MANUAL bulk caching

            // Turret auto-align: gamepad2 A toggles, gamepad2 RB / Back rezero odometry to the loading-zone corner (facing +x) / far corner (facing +x)
            if (kGamePad2.isButtonAFirstPressed()) {
                autoAlign = !autoAlign;
                turretAutoAlignTeleOp.setTurretRunMode(autoAlign ? TurretRunMode.RUN_USING_ODOMETRY : TurretRunMode.STOP);
                if (!autoAlign) {
                    turret1Position = turret1Servo.getPosition(); // dpad resumes from where auto-align left it
                }
                final boolean on = autoAlign;
                KLog.d(TAG, () -> "event=autoAlign on=" + on);
            }
            if (kGamePad2.isRightBumperFirstPressed()) {
                setLastZeroAction(new ResetOdometryToPos(new Position(FieldConfig.CORNER_ZERO_POSE)));
                KLog.d(TAG, () -> "event=rezero to=" + FieldConfig.CORNER_ZERO_POSE);
            }
            if (kGamePad2.isBackButtonFirstPressed()) {
                setLastZeroAction(new ResetOdometryToPos(new Position(FieldConfig.FAR_CORNER_ZERO_POSE)));
                KLog.d(TAG, () -> "event=rezero to=" + FieldConfig.FAR_CORNER_ZERO_POSE);
            }
            updateActions();


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

            // Stopper positions (gamepad2 dpad, held): up/down = open +/-, right/left = closed +/-.
            // The stopper moves to the position being edited. Ignored while a push sequence runs.
            if (pushState == PushState.IDLE) {
                if (gamepad2.dpad_up || gamepad2.dpad_down) {
                    stopperOpenPos = Math.max(0, Math.min(1, stopperOpenPos + (gamepad2.dpad_up ? SERVO_STEP : -SERVO_STEP)));
                    stopperServo.setPosition(stopperOpenPos);
                } else if (gamepad2.dpad_right || gamepad2.dpad_left) {
                    stopperClosedPos = Math.max(0, Math.min(1, stopperClosedPos + (gamepad2.dpad_right ? SERVO_STEP : -SERVO_STEP)));
                    stopperServo.setPosition(stopperClosedPos);
                }
            }

            // 0.404 OPEN
            // 0.440 CLOSED

            // B (first press): push sequence. Ignored while one is already running.
            boolean bPressed = gamepad1.b && !prevB;
            prevB = gamepad1.b;
            if (bPressed && pushState == PushState.IDLE) {
                stopperServo.setPosition(stopperClosedPos);
                pushTimer.reset();
                pushState = PushState.PRE_OPEN;
            }
            if (pushState == PushState.PRE_OPEN && pushTimer.milliseconds() >= PRE_OPEN_MS) {
                stopperServo.setPosition(stopperOpenPos);
                pushTimer.reset();
                pushState = PushState.SHOOTING;
            }
            if (pushState == PushState.SHOOTING && pushTimer.milliseconds() >= SHOOT_MS) {
                stopperServo.setPosition(stopperClosedPos);
                pushState = PushState.IDLE;
            }

            if (pushState != PushState.IDLE) {
                intakeMotor1.setPower(intakePower);
                intakeMotor2.setPower(-intakePower);
            } else if (gamepad1.right_bumper) {
                stopperServo.setPosition(stopperClosedPos);
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
                stopperServo.setPosition(stopperClosedPos);
            } else if (gamepad1.y) {
                stopperPosition -= SERVO_STEP;
                stopperServo.setPosition(stopperOpenPos);
            }

            // Turret 1 (servo2): dpad right / left (auto-align off only)
            if (autoAlign) {
                // TurretAutoAlign owns the turret
            } else if (gamepad1.dpad_right) {
                turret1Position = Math.min(1, turret1Position + SERVO_STEP);
                turret1Servo.setPosition(turret1Position);
            } else if (gamepad1.dpad_left) {
                turret1Position = Math.max(0, turret1Position - SERVO_STEP);
                turret1Servo.setPosition(turret1Position);
            }

            // Intake pivots (gamepad2 sticks, held): left stick Y = pivot1, right stick Y = pivot2 (up = +)
            if (Math.abs(gamepad2.left_stick_y) > 0.5) {
                intakePivot1Position = Math.max(0, Math.min(1, intakePivot1Position + (gamepad2.left_stick_y < 0 ? SERVO_STEP : -SERVO_STEP)));
                intakePivot1Servo.setPosition(intakePivot1Position);
            }
            if (Math.abs(gamepad2.right_stick_y) > 0.5) {
                intakePivot2Position = Math.max(0, Math.min(1, intakePivot2Position + (gamepad2.right_stick_y < 0 ? SERVO_STEP : -SERVO_STEP)));
                intakePivot2Servo.setPosition(intakePivot2Position);
            }

            if (gamepad1.dpad_right || gamepad1.dpad_left
                    || Math.abs(gamepad2.left_stick_y) > 0.5 || Math.abs(gamepad2.right_stick_y) > 0.5) {
                final double p1 = intakePivot1Position, t1 = turret1Position, p2 = intakePivot2Position;
                KLog.d(TAG, () -> String.format("event=servoMove intakePivot1=%.4f turret1=%.4f intakePivot2=%.4f", p1, t1, p2));
            }


            telemetry.addLine("--- Controls (gamepad1) ---");
            telemetry.addLine("Shooter (motor0): left stick Y (up = -, down = +), X = stop");
            telemetry.addLine("Intake: RB = in, LB = out, RT / LT = power up / down");
            telemetry.addLine("Hood (servo0): dpad up / down");
            telemetry.addLine("Stopper (servo1): A = close, Y = open");
            telemetry.addLine("Stopper positions (gamepad2 dpad, hold): up/down = open +/-, right/left = closed +/-");
            telemetry.addData("Stopper OPEN pos (copy to ModuleConfig)", "%.3f", stopperOpenPos);
            telemetry.addData("Stopper CLOSED pos (copy to ModuleConfig)", "%.3f", stopperClosedPos);
            telemetry.addLine("Turret 1 (servo2): dpad right / left (auto-align off)");
            telemetry.addLine("Turret auto-align (gamepad2): A = toggle, RB = rezero odometry (loading-zone corner, facing +x), Back = far corner, facing +x");
            telemetry.addData("Auto-align on / aligned", "%b / %b", autoAlign, turretAutoAlignTeleOp.isWithinRange());
            telemetry.addData("Turret target / delta", "%.1f / %.1f deg",
                    Math.toDegrees(turretAutoAlignTeleOp.getTargetAngleRad()), turretAutoAlignTeleOp.getDeltaAngleDeg());
            telemetry.addData("Odometry pose", SharedData.getOdometryWheelIMUPosition());
            telemetry.addLine("Turret 2 (servo3): right bumper / left bumper");
            telemetry.addLine("Intake pivots (gamepad2, hold): left stick Y = pivot1 (CH0), right stick Y = pivot2 (CH2)");
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
            telemetry.addData("Intake pivot 1 (CH0)", "%.4f / %.4f", intakePivot1Position, intakePivot1Servo.getPosition());
            telemetry.addData("Intake pivot 2 (CH2)", "%.4f / %.4f", intakePivot2Position, intakePivot2Servo.getPosition());
            telemetry.update();
        }
        cleanupRobot();
    }
}
