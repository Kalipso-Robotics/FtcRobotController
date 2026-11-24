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
 * Finds the hood hard limits ShooterConfig.HOOD_DOWN_POS / HOOD_UP_POS (all the way down / up).
 * Raw position goes UP as the hood goes UP. ShooterConfig.hoodFlipDirection makes that true.
 *
 * GP1: dpad up = hood up, dpad down = hood down (hold RB for fine steps).
 *      A = mark LOWEST, Y = mark HIGHEST. Telemetry prints the two lines to paste into ShooterConfig.
 * If dpad up moves the hood DOWN, flip ShooterConfig.hoodFlipDirection (Dashboard, applies live) and re-mark.
 * Range is the full servo 0..1, and bypasses the KServo hard limits, so you can find limits outside the old values.
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
        hood.getServo().setPosition(pos); // bypass hard limits to find them
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
            hood.getServo().setPosition(pos); // bypass hard limits to find them
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
            telemetry.addData("now", "HOOD_DOWN_POS %.4f, HOOD_UP_POS %.4f", ShooterConfig.HOOD_DOWN_POS, ShooterConfig.HOOD_UP_POS);
            telemetry.addLine(Double.isNaN(lowest) ? "lowest: press A at the bottom"
                    : String.format("HOOD_DOWN_POS = %.4f; // ShooterConfig", lowest));
            telemetry.addLine(Double.isNaN(highest) ? "highest: press Y at ~90 deg up"
                    : String.format("HOOD_UP_POS = %.4f; // ShooterConfig", highest));
            if (!Double.isNaN(lowest) && !Double.isNaN(highest) && highest <= lowest) {
                telemetry.addLine("!! highest <= lowest: flip hoodFlipDirection and re-mark");
            }
            telemetry.update();
        }
    }
}
