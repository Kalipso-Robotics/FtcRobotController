package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig.hoodFlipDirection;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Moves the hood in small steps to find its real limits. Starts at MIN_HOOD and is clamped to MIN_HOOD..MAX_HOOD
 * (raw servo values, hoodFlipDirection applied by KServo). GP1 dpad down = +STEP, dpad up = -STEP.
 */
@TeleOp
public class HoodServoToZero extends LinearOpMode {
    public static final String TAG = "Test_Hood";
    private static final double STEP = 0.0001;

    private double hoodPos = ShooterInterpolationConfig.MIN_HOOD;

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        Servo hood = opModeUtilities.getHardwareMap().servo.get("hood");
        KServo khood = new KServo(hood, KServo.AXON_MAX_SPEED_DEG_PER_SEC, 255, hoodPos, hoodFlipDirection);
        khood.setPosition(hoodPos);

        waitForStart();
        while (opModeIsActive()) {
            double before = hoodPos;
            if (gamepad1.dpad_down) {
                hoodPos += STEP;
            } else if (gamepad1.dpad_up) {
                hoodPos -= STEP;
            }
            hoodPos = MathFunctions.clamp(hoodPos, ShooterInterpolationConfig.MIN_HOOD, ShooterInterpolationConfig.MAX_HOOD);
            khood.setPosition(hoodPos);
            if (hoodPos != before) {
                double p = hoodPos;
                KLog.d(TAG, () -> "event=hoodSet pos=" + p);
            }
            telemetry.addLine("dpad down = +" + STEP + ", dpad up = -" + STEP);
            telemetry.addData("hood pos", "%.4f (range %.4f..%.4f)", hoodPos, ShooterInterpolationConfig.MIN_HOOD, ShooterInterpolationConfig.MAX_HOOD);
            telemetry.addData("servo reads", "%.4f", khood.getPosition());
            telemetry.update();
        }
    }
}
