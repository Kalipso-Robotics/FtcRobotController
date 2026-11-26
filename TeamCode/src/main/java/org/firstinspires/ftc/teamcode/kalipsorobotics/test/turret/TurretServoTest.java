package org.firstinspires.ftc.teamcode.kalipsorobotics.test.turret;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;

/**
 * Hand-driven check of the turret servo. Pattern source: decode/calibrations/StopperCalibration (KOpMode skeleton).
 * GP2: dpad left/right = -/+5 deg, hold either bumper + dpad = -/+0.5 deg (finding true zero), A = 0, X = +170, Y = -170.
 * Targets past the limit show up as isClamped.
 * Range check: dpad to 90 deg; if the turret doesn't turn a true 90,
 * true TurretConfig.SERVO_RANGE_DEG = current * actual turret deg / requested deg.
 * Finding true zero: INIT holds SERVO_ZERO_POS; nudge with bumper + dpad until the turret points straight ahead, then
 * copy the servoPos telemetry into TurretConfig.SERVO_ZERO_POS. Positive deg = turret turns right (clockwise from above).
 */
@TeleOp(name = "TurretServoTest", group = "Test")
public class TurretServoTest extends KOpMode {
    public static final String TAG = "Test_Turret";
    private static final double STEP_DEG = 5;
    private static final double FINE_STEP_DEG = 0.5;
    private static final double EXTREME_DEG = 170;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        Turret turret = Turret.getInstance(opModeUtilities);
        double targetDeg = 0;
        waitForStart();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            boolean changed = true;
            double step = (gamepad2.left_bumper || gamepad2.right_bumper) ? FINE_STEP_DEG : STEP_DEG;
            if (kGamePad2.isDpadRightFirstPressed()) {
                targetDeg += step;
            } else if (kGamePad2.isDpadLeftFirstPressed()) {
                targetDeg -= step;
            } else if (kGamePad2.isButtonAFirstPressed()) {
                targetDeg = 0;
            } else if (kGamePad2.isButtonXFirstPressed()) {
                targetDeg = EXTREME_DEG;
            } else if (kGamePad2.isButtonYFirstPressed()) {
                targetDeg = -EXTREME_DEG;
            } else {
                changed = false;
            }
            if (changed) {
                turret.setAngleDeg(targetDeg);
                double requested = targetDeg;
                KLog.d(TAG, () -> String.format("event=target requestedDeg=%.1f commandedDeg=%.1f servoPos=%.4f clamped=%s",
                        requested, turret.getAngleDeg(), Turret.servoPosFromTurretDeg(turret.getAngleDeg()), turret.isClamped()));
            }

            telemetry.addLine("GP2 dpad L/R: -/+5 deg   hold bumper + dpad: -/+0.5 deg   A: zero");
            telemetry.addLine("Not a true 90? SERVO_RANGE_DEG = current * actual / requested");
            telemetry.addLine("Straight ahead? Put servoPos into TurretConfig.SERVO_ZERO_POS");
            telemetry.addData("servo range", "%.0f deg", TurretConfig.SERVO_RANGE_DEG);
            telemetry.addData("requestedDeg", "%.1f", targetDeg);
            telemetry.addData("commandedDeg", "%.1f", turret.getAngleDeg());
            telemetry.addData("servoPos", "%.4f", Turret.servoPosFromTurretDeg(turret.getAngleDeg()));
            telemetry.addData("isClamped", turret.isClamped());
            telemetry.addData("isMoving", turret.isMoving());
            telemetry.update();
        }
        cleanupRobot();
    }
}
