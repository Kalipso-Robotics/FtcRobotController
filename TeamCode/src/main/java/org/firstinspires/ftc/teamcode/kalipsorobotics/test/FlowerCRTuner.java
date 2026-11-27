package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;

/**
 * Tunes the flower arm's servo positions (FlowerMech.UP_POS / DOWN_POS). "flowerPivot" is a KServo now (Axon in
 * servo mode, configured as a Servo), so this sets positions instead of CR power.
 * Pattern source: IntakeMotorTest. GP1:
 *   A / Y          = lower / raise the flower's intake side (flower can only go down while the side is down)
 *   dpad up / down = nudge the flower position +/- POS_STEP (only while the side is down, except back to UP_POS)
 *   X / B          = flower to FlowerMech.DOWN_POS / UP_POS
 * Nudge to the real up and down stops, then copy the shown positions into FlowerMech.
 */
@TeleOp(name = "FlowerCRTuner", group = "Test")
public class FlowerCRTuner extends KOpMode {
    public static final String TAG = "Test_Flower";
    private static final double POS_STEP = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        Intake intake = new Intake(opModeUtilities);
        FlowerMech flower = intake.getFlower();
        IntakeModule side = intake.getModule(ModuleConfig.FLOWER_SIDE);
        ElapsedTime sinceCommand = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        sinceCommand.reset();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();

            if (kGamePad1.isButtonAFirstPressed()) {
                side.lower();
            } else if (kGamePad1.isButtonYFirstPressed()) {
                side.raise(); // raises the flower first
                sinceCommand.reset();
            }

            boolean nudgeUp = kGamePad1.isDpadUpFirstPressed();
            boolean nudgeDown = kGamePad1.isDpadDownFirstPressed();
            if (nudgeUp || nudgeDown) {
                double target = MathFunctions.clamp(flower.getServo().getPosition() + (nudgeUp ? POS_STEP : -POS_STEP), 0, 1);
                boolean ok = target == FlowerMech.UP_POS || side.isDown(); // interlock: only leaves up while the side is down
                if (ok) {
                    flower.getServo().setTargetPosition(target);
                }
                sinceCommand.reset();
                KLog.d(TAG, () -> "event=nudge accepted=" + ok + " pos=" + target);
            }

            if (kGamePad1.isButtonXFirstPressed()) {
                boolean ok = side.isDown();
                if (ok) {
                    flower.getServo().setTargetPosition(FlowerMech.DOWN_POS);
                }
                sinceCommand.reset();
                KLog.d(TAG, () -> "event=lower accepted=" + ok + " pos=" + FlowerMech.DOWN_POS);
            } else if (kGamePad1.isButtonBFirstPressed()) {
                flower.getServo().setTargetPosition(FlowerMech.UP_POS);
                sinceCommand.reset();
                KLog.d(TAG, () -> "event=raise pos=" + FlowerMech.UP_POS);
            }
            intake.update();

            telemetry.addLine("A/Y side down/up | dpad up/down nudge flower | X/B flower down/up");
            telemetry.addData("side", "%s %s", ModuleConfig.FLOWER_SIDE, side.isDown() ? "DOWN" : side.isUp() ? "UP" : "moving");
            telemetry.addData("flower pos", "%.3f%s", flower.getServo().getPosition(), flower.getServo().isDone() ? "" : " (moving)");
            telemetry.addData("flower", flower.isDown() ? "DOWN" : flower.isUp() ? "UP" : "moving");
            telemetry.addData("ms since command", "%.0f", sinceCommand.milliseconds());
            telemetry.addData("FlowerMech", "UP_POS %.3f  DOWN_POS %.3f", FlowerMech.UP_POS, FlowerMech.DOWN_POS);
            telemetry.update();
        }
        intake.stop();
        cleanupRobot();
    }
}
