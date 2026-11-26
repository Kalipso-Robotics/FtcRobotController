package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;

/**
 * Tunes the CR flower arm (IntakeConfig.FLOWER_DOWN_POWER / FLOWER_UP_POWER / FLOWER_LOWER_MS /
 * FLOWER_RAISE_MS / FLOWER_FLIP_DIRECTION). Start with the arm up by hand.
 * Pattern source: IntakeMotorTest. GP1:
 *   A / Y      = lower / raise the flower's intake side (flower can only go down while the side is down)
 *   left stick = raw flower power (up on the stick = positive). Find the sign that goes down and a power that moves well.
 *   X / B      = flower.lower() / raise(): the timed moves with the IntakeConfig values. Adjust the two times live on the
 *                Dashboard until each move ends right at the end of travel.
 * Then copy the values into IntakeConfig. FLOWER_FLIP_DIRECTION is read at init, so restart after changing it.
 */
@TeleOp(name = "FlowerCRTuner", group = "Test")
public class FlowerCRTuner extends KOpMode {
    public static final String TAG = "Test_Flower";
    private static final double STICK_DEADBAND = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        Intake intake = new Intake(opModeUtilities);
        FlowerMech flower = intake.getFlower();
        IntakeModule side = intake.getModule(ModuleConfig.FLOWER_SIDE);
        ElapsedTime sinceCommand = new ElapsedTime();
        boolean manual = false;
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

            double stick = -gamepad1.left_stick_y;
            if (Math.abs(stick) > STICK_DEADBAND) {
                if (!manual) {
                    sinceCommand.reset();
                }
                manual = true;
                flower.setPower(stick);
            } else if (manual) {
                manual = false;
                flower.setPower(0);
            }

            if (kGamePad1.isButtonXFirstPressed()) {
                boolean ok = flower.lower();
                sinceCommand.reset();
                KLog.d(TAG, () -> "event=lower accepted=" + ok + " power=" + IntakeConfig.FLOWER_DOWN_POWER);
            } else if (kGamePad1.isButtonBFirstPressed()) {
                flower.raise();
                sinceCommand.reset();
                KLog.d(TAG, () -> "event=raise power=" + IntakeConfig.FLOWER_UP_POWER);
            }
            intake.update();

            telemetry.addLine("A/Y side down/up | stick raw power | X/B flower down/up (config)");
            telemetry.addData("side", "%s %s", ModuleConfig.FLOWER_SIDE, side.isDown() ? "DOWN" : side.isUp() ? "UP" : "moving");
            telemetry.addData("flower power", "%.2f%s", flower.getPower(), manual ? " (manual)" : "");
            telemetry.addData("flower", flower.isDown() ? "DOWN" : flower.isUp() ? "UP" : "moving");
            telemetry.addData("ms since command", "%.0f", sinceCommand.milliseconds());
            telemetry.addData("config", "down %.2f  up %.2f  lower %.0f ms  raise %.0f ms  flip %b", IntakeConfig.FLOWER_DOWN_POWER,
                    IntakeConfig.FLOWER_UP_POWER, IntakeConfig.FLOWER_LOWER_MS, IntakeConfig.FLOWER_RAISE_MS, IntakeConfig.FLOWER_FLIP_DIRECTION);
            telemetry.update();
        }
        flower.setPower(0);
        intake.stop();
        cleanupRobot();
    }
}
