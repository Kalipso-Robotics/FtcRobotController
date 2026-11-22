package org.firstinspires.ftc.teamcode.kalipsorobotics.test.turret;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;

/**
 * Hand-driven check of the turret servo. Pattern source: decode/calibrations/StopperCalibration (KOpMode skeleton).
 * GP2: dpad left/right = -/+5 deg, A = 0, X = +170, Y = -170. Targets past the limit show up as isClamped.
 */
@TeleOp(name = "TurretServoTest", group = "Test")
public class TurretServoTest extends KOpMode {
    public static final String TAG = "Test_Turret";
    private static final double STEP_DEG = 5;
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
            if (kGamePad2.isDpadRightFirstPressed()) {
                targetDeg += STEP_DEG;
            } else if (kGamePad2.isDpadLeftFirstPressed()) {
                targetDeg -= STEP_DEG;
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
                KLog.d(TAG, () -> String.format("event=target requestedDeg=%.1f commandedDeg=%.1f clamped=%s",
                        requested, turret.getAngleDeg(), turret.isClamped()));
            }

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
