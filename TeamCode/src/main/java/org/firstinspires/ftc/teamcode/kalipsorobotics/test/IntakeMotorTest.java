package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;

/**
 * Hand-driven check of one intake side at a time. Pattern source: decode/calibrations/StopperCalibration (KOpMode skeleton).
 * GP2: dpad up/down = side A/B, A = lower, Y = raise, RT = run, LT = reverse, B = flower toggle.
 * Flower interlock stays in Flower/IntakeSide; this only calls their methods and logs the result.
 */
@TeleOp(name = "IntakeMotorTest", group = "Test")
public class IntakeMotorTest extends KOpMode {
    public static final String TAG = "Test_Intake";

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        Intake intake = new Intake(opModeUtilities);
        FlowerMech flowerMech = intake.getFlower();
        BeamBreakPosition.IntakeSide selected = BeamBreakPosition.IntakeSide.A;
        waitForStart();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            BeamBreakPosition.IntakeSide previous = selected;
            if (kGamePad2.isDpadUpFirstPressed()) {
                selected = BeamBreakPosition.IntakeSide.A;
            } else if (kGamePad2.isDpadDownFirstPressed()) {
                selected = BeamBreakPosition.IntakeSide.B;
            }
            if (selected != previous) {
                intake.getSide(previous).stop();
                BeamBreakPosition.IntakeSide now = selected;
                KLog.d(TAG, () -> "event=selectSide side=" + now);
            }
            IntakeSide side = intake.getSide(selected);

            if (kGamePad2.isButtonAFirstPressed()) {
                side.lower();
                KLog.d(TAG, () -> "event=lower side=" + side.getSide());
            } else if (kGamePad2.isButtonYFirstPressed()) {
                side.raise();
                KLog.d(TAG, () -> "event=raise side=" + side.getSide());
            }
            if (kGamePad2.isButtonBFirstPressed()) {
                if (flowerMech.isDown()) {
                    flowerMech.raise();
                    KLog.d(TAG, "event=flowerRaise");
                } else {
                    boolean lowered = flowerMech.lower();
                    KLog.d(TAG, () -> "event=flowerLower accepted=" + lowered);
                }
            }

            if (kGamePad2.isRightTriggerPressed()) {
                side.run(1);
            } else if (kGamePad2.isLeftTriggerPressed()) {
                side.run(-1);
            } else {
                side.stop();
            }
            intake.update();

            telemetry.addData("side", selected);
            telemetry.addData("pivotPos", "%.3f", side.getPivotPosition());
            telemetry.addData("isDown", side.isDown());
            telemetry.addData("power", "%.2f", side.getPower());
            telemetry.addData("ballCount", intake.getBallCount());
            telemetry.addData("flowerDown", flowerMech.isDown());
            telemetry.addData("flowerUp", flowerMech.isUp());
            telemetry.update();
        }
        intake.stop();
        cleanupRobot();
    }
}
