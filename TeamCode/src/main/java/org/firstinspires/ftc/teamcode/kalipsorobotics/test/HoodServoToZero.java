package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Re-zero the hood. Convention (same as Shooter / ShooterRun / HoodSetAction):
 *   MIN_HOOD = hood all the way DOWN (lowest), MAX_HOOD = hood all the way UP (~90 deg),
 *   raw position goes UP as the hood goes UP. ShooterConfig.hoodFlipDirection makes that true.
 *
 * GP1: dpad up = hood up, dpad down = hood down (hold RB for fine steps).
 *      A = mark LOWEST, Y = mark HIGHEST. Telemetry prints the two lines to paste into ShooterInterpolationConfig.
 * If dpad up moves the hood DOWN, flip ShooterConfig.hoodFlipDirection (Dashboard, applies live) and re-mark.
 * Range is the full servo 0..1, not MIN_HOOD..MAX_HOOD, so you can find limits outside the old values.
 * The flywheel is never powered.
 */
@TeleOp
public class HoodServoToZero extends LinearOpMode {
    private static final double STEP = 0.001;      // ~0.05/s at 50 Hz
    private static final double FINE_STEP = 0.0001;

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        KServo hood = new Shooter(opModeUtilities).getHood();
        double pos = ShooterInterpolationConfig.MIN_HOOD;
        double lowest = Double.NaN;
        double highest = Double.NaN;

        waitForStart();
        hood.setPosition(pos);
        while (opModeIsActive()) {
            hood.setDirection(ShooterConfig.hoodFlipDirection ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

            double step = gamepad1.right_bumper ? FINE_STEP : STEP;
            double before = pos;
            if (gamepad1.dpad_up) {
                pos += step;
            } else if (gamepad1.dpad_down) {
                pos -= step;
            }
            pos = MathFunctions.clamp(pos, 0, 1);
            hood.setPosition(pos);
            if (pos != before) {
                double p = pos;
                KLog.d(Shooter.HOOD_TAG, () -> "event=zeroJog pos=" + p);
            }

            if (gamepad1.a && pos != lowest) {
                lowest = pos;
                double p = pos;
                KLog.d(Shooter.HOOD_TAG, () -> "event=zeroMarkLowest pos=" + p);
            }
            if (gamepad1.y && pos != highest) {
                highest = pos;
                double p = pos;
                KLog.d(Shooter.HOOD_TAG, () -> "event=zeroMarkHighest pos=" + p);
            }

            telemetry.addLine("dpad up = hood UP, dpad down = hood DOWN, hold RB = fine");
            telemetry.addLine("A = mark LOWEST, Y = mark HIGHEST (~90 deg up)");
            telemetry.addLine("dpad up moves hood DOWN? flip ShooterConfig.hoodFlipDirection");
            telemetry.addData("hood pos", "%.4f", pos);
            telemetry.addData("now", "MIN_HOOD %.4f (lowest), MAX_HOOD %.4f (highest)",
                    ShooterInterpolationConfig.MIN_HOOD, ShooterInterpolationConfig.MAX_HOOD);
            telemetry.addLine(Double.isNaN(lowest) ? "lowest: press A at the bottom"
                    : String.format("MIN_HOOD = %.4f; // lowest hood", lowest));
            telemetry.addLine(Double.isNaN(highest) ? "highest: press Y at ~90 deg up"
                    : String.format("MAX_HOOD = %.4f; // highest hood, ~90 deg up", highest));
            if (!Double.isNaN(lowest) && !Double.isNaN(highest) && highest <= lowest) {
                telemetry.addLine("!! highest <= lowest: flip hoodFlipDirection and re-mark");
            }
            telemetry.update();
        }
    }
}
