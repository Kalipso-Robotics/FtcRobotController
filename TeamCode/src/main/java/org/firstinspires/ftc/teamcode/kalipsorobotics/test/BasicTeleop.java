package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakArray;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * Fully manual control of every piece of hardware, for bring-up (no Actions, no hardwareMap: everything goes through the
 * module methods, so the Flower/intake interlock still applies and shows on screen when it refuses).
 * Pattern source: test/turret/TurretServoTest and decode/calibrations/StopperCalibration.
 *
 * GP1: left stick + right stick X = robot-centric drive | RT hold = intakes in, LT hold = intakes reverse
 *      dpad up/down = shooter power +/- BASIC_SHOOTER_STEP (held until changed), back = shooter 0
 *      RB / LB = next / previous servo | X / Y / B = selected servo to 0.0 / 0.5 / 1.0
 *      dpad right/left = selected servo +/- BASIC_SERVO_STEP (with A held: BASIC_SERVO_FINE_STEP)
 * Steps live in ModuleConfig BASIC_*.
 */
@TeleOp(name = "BasicTeleop", group = "Test")
public class BasicTeleop extends KOpMode {
    public static final String TAG = "Test_Basic";

    private static final String[] SERVOS = {"pivotA", "pivotB", "stopper", "hood", "turret"};

    private DriveTrain driveTrain;
    private Intake intake;
    private IntakeModule sideA;
    private IntakeModule sideB;
    private Stopper stopper;
    private Shooter shooter;
    private Turret turret;
    private BeamBreakArray beamBreakArray;

    private int selected = 0;
    private double shooterPower = 0;
    private String lastRefusal = "none";

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);
        intake = new Intake(opModeUtilities);
        sideA = intake.getModule(BeamBreakPosition.IntakeSide.A);
        sideB = intake.getModule(BeamBreakPosition.IntakeSide.B);
        stopper = new Stopper(opModeUtilities);
        shooter = new Shooter(opModeUtilities);
        beamBreakArray = new BeamBreakArray(opModeUtilities);
        OpModeUtilities.startOdometry(opModeUtilities, odoExecutorService, driveTrain, imuModule, new Position(0, 0, 0));

        telemetry.addLine("BasicTeleop: manual control of every motor and servo. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            beamBreakArray.update();
            intake.update();

            drive();
            runIntakes();
            runShooter();
            selectServo();
            moveServo();

            renderTelemetry();
        }
        driveTrain.setPower(0);
        intake.stop();
        shooter.stop();
        cleanupRobot();
    }

    /** Robot-centric mecanum: left stick = translate, right stick X = turn. */
    private void drive() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double fLeft = forward + strafe + turn;
        double fRight = forward - strafe - turn;
        double bLeft = forward - strafe + turn;
        double bRight = forward + strafe - turn;
        double max = Math.max(1, Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight))));
        double k = ModuleConfig.BASIC_DRIVE_SCALE / max;
        driveTrain.setPower(fLeft * k, fRight * k, bLeft * k, bRight * k);
    }

    private void runIntakes() {
        double power = 0;
        if (gamepad1.right_trigger > 0.1) {
            power = IntakeConfig.intakePower;
        } else if (gamepad1.left_trigger > 0.1) {
            power = -IntakeConfig.intakePower;
        }
        sideA.run(power);
        sideB.run(power);
    }

    private void runShooter() {
        double before = shooterPower;
        if (kGamePad1.isDpadUpFirstPressed()) {
            shooterPower += ModuleConfig.BASIC_SHOOTER_STEP;
        } else if (kGamePad1.isDpadDownFirstPressed()) {
            shooterPower -= ModuleConfig.BASIC_SHOOTER_STEP;
        }
        if (kGamePad1.isBackButtonFirstPressed()) {
            shooterPower = 0;
        }
        shooterPower = MathFunctions.clamp(shooterPower, 0, 1);
        shooter.setPower(shooterPower);
        if (shooterPower != before) {
            double p = shooterPower;
            KLog.d(TAG, () -> "event=shooterSet power=" + p);
        }
    }

    private void selectServo() {
        if (kGamePad1.isRightBumperFirstPressed()) {
            selected = (selected + 1) % SERVOS.length;
        } else if (kGamePad1.isLeftBumperFirstPressed()) {
            selected = (selected + SERVOS.length - 1) % SERVOS.length;
        }
    }

    private void moveServo() {
        double current = servoPosition(selected);
        Double target = null;
        if (kGamePad1.isButtonXFirstPressed()) {
            target = 0.0;
        } else if (kGamePad1.isButtonYFirstPressed()) {
            target = 0.5;
        } else if (kGamePad1.isButtonBFirstPressed()) {
            target = 1.0;
        } else {
            double step = gamepad1.a ? ModuleConfig.BASIC_SERVO_FINE_STEP : ModuleConfig.BASIC_SERVO_STEP;
            if (kGamePad1.isDpadRightFirstPressed()) {
                target = current + step;
            } else if (kGamePad1.isDpadLeftFirstPressed()) {
                target = current - step;
            }
        }
        if (target != null) {
            setServo(selected, MathFunctions.clamp(target, 0, 1));
        }
    }

    private void setServo(int index, double pos) {
        String name = SERVOS[index];
        boolean ok = true;
        switch (name) {
            case "pivotA":
                ok = sideA.setPivotPosition(pos);
                break;
            case "pivotB":
                ok = sideB.setPivotPosition(pos);
                break;
            case "stopper":
                stopper.getStopper().setTargetPosition(pos); // keeps the KServo target (shown on screen) in sync
                break;
            case "hood":
                pos = MathFunctions.clamp(pos, ShooterInterpolationConfig.MIN_HOOD, ShooterInterpolationConfig.MAX_HOOD);
                shooter.getHood().setTargetPosition(pos);
                break;
            default: // turret: no direct servo access, go through its angle (Turret clamps to the reachable range)
                turret.setAngleDeg(Turret.turretDegFromServoPos(pos));
                break;
        }
        double logged = pos;
        if (ok) {
            KLog.d(TAG, () -> "event=servoSet name=" + name + " pos=" + logged);
        } else {
            lastRefusal = name + " to " + String.format("%.3f", pos) + " refused by the interlock"
                    + " (raise the flower first)";
            KLog.d(TAG, () -> "event=servoRefused name=" + name + " pos=" + logged);
        }
    }

    private double servoPosition(int index) {
        switch (SERVOS[index]) {
            case "pivotA":
                return sideA.getPivotPosition();
            case "pivotB":
                return sideB.getPivotPosition();
            case "stopper":
                return stopper.getStopper().getTargetPosition();
            case "hood":
                return shooter.getHood().getTargetPosition();
            default:
                return Turret.servoPosFromTurretDeg(turret.getAngleDeg());
        }
    }

    private void renderTelemetry() {
        telemetry.addLine("sticks drive | RT in / LT reverse | dpad U/D shooter +/-, back = 0");
        telemetry.addLine("RB/LB servo | X/Y/B = 0 / 0.5 / 1 | dpad R/L step (A held = fine)");
        for (int i = 0; i < SERVOS.length; i++) {
            telemetry.addData((i == selected ? "> " : "  ") + SERVOS[i], "%.3f", servoPosition(i));
        }
        telemetry.addData("turret deg", "%.1f", turret.getAngleDeg());
        telemetry.addData("shooter", "power %.2f  rps %.1f", shooterPower, shooter.getRPS());
        telemetry.addData("intake A", "power %.2f  %s", sideA.getPower(), sideA.isDown() ? "DOWN" : sideA.isUp() ? "UP" : "moving");
        telemetry.addData("intake B", "power %.2f  %s", sideB.getPower(), sideB.isDown() ? "DOWN" : sideB.isUp() ? "UP" : "moving");
        telemetry.addData("flower", intake.getFlower().isDown() ? "DOWN" : intake.getFlower().isUp() ? "UP" : "moving");
        telemetry.addData("drive", "fL %.2f fR %.2f bL %.2f bR %.2f",
                driveTrain.getfLeftPower(), driveTrain.getfRightPower(), driveTrain.getbLeftPower(), driveTrain.getbRightPower());
        Position pose = SharedData.getOdometryWheelIMUPosition();
        telemetry.addData("odometry", "x %.0f  y %.0f  h %.1f deg", pose.getX(), pose.getY(), Math.toDegrees(pose.getTheta()));
        boolean reliable = beamBreakArray.isPatternPlausible() && !beamBreakArray.isSettling();
        telemetry.addData("balls", "%d (%s)", beamBreakArray.getCount(), reliable ? "reliable" : "settling/corrected");
        telemetry.addData("last refusal", lastRefusal);
        telemetry.update();
    }
}
