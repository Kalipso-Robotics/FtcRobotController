package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeFullDetector;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;

/**
 * Hand-driven check of one intake side at a time. Pattern source: decode/calibrations/StopperCalibration (KOpMode skeleton).
 * The stopper is closed at start (like TeleOp) so balls pack against it.
 * GP2: dpad up/down = side A/B, A = lower, Y = raise, RT = run, LT = reverse, B = flower toggle,
 * X (hold) = selected side closed loop at IntakeConfig.FEED_RPS (the shot feed; tune it in IntakeTest).
 * Flower interlock stays in Flower/IntakeModule; this only calls their methods and logs the result.
 *
 * Also the IntakeFullDetector tuner (all thresholds in IntakeConfig, live on the Dashboard): hold RT and add one ball
 * at a time. filteredMA is graphed; set BALL_1..4_MA a bit below each step. Packed against the closed stopper, set
 * STALL_CURRENT_MA below that current and STALL_RPS above that RPS. "peak" is the highest filteredMA this run.
 */
@TeleOp(name = "IntakeMotorTest", group = "Test")
public class IntakeMotorTest extends KOpMode {
    public static final String TAG = "Test_Intake";

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        Intake intake = new Intake(opModeUtilities);
        FlowerMech flowerMech = intake.getFlower();
        Stopper stopper = new Stopper(opModeUtilities);
        IntakeSide selected = IntakeSide.FRONT;
        IntakeFullDetector fullDetector = new IntakeFullDetector();
        boolean wasRunning = false;
        double peakFilteredMA = 0;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        stopper.setPosition(ModuleConfig.STOPPER_SERVO_CLOSED_POS); // balls must pack against it for the stall to show

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            IntakeSide previous = selected;
            if (kGamePad2.isDpadUpFirstPressed()) {
                selected = IntakeSide.FRONT;
            } else if (kGamePad2.isDpadDownFirstPressed()) {
                selected = IntakeSide.BACK;
            }
            if (selected != previous) {
                intake.getModule(previous).stop();
                IntakeSide now = selected;
                KLog.d(TAG, () -> "event=selectSide side=" + now);
            }
            IntakeModule side = intake.getModule(selected);

            if (kGamePad2.isButtonAFirstPressed()) {
                side.lower();
                KLog.d(TAG, () -> "event=lower side=" + side.getSide());
            } else if (kGamePad2.isButtonYFirstPressed()) {
                side.raise();
                KLog.d(TAG, () -> "event=raise side=" + side.getSide());
            }
            if (kGamePad2.isButtonBFirstPressed()) {
                if (flowerMech.isDown()) {
                    flowerMech.getServo().setTargetPosition(FlowerMech.UP_POS);
                    KLog.d(TAG, "event=flowerRaise");
                } else {
                    // interlock: only lower while the flower's intake side is down
                    boolean lowered = intake.getModule(ModuleConfig.FLOWER_SIDE).isDown();
                    if (lowered) {
                        flowerMech.getServo().setTargetPosition(FlowerMech.DOWN_POS);
                    }
                    KLog.d(TAG, () -> "event=flowerLower accepted=" + lowered);
                }
            }

            boolean running = kGamePad2.isRightTriggerPressed();
            if (kGamePad2.isXPressed()) {
                side.runRPS(IntakeConfig.FEED_RPS);
            } else if (running) {
                side.run(1);
            } else if (kGamePad2.isLeftTriggerPressed()) {
                side.run(-1);
            } else {
                side.stop();
            }
            intake.update();

            long now = System.currentTimeMillis();
            if (running && !wasRunning) {
                fullDetector.reset(now);
                peakFilteredMA = 0;
            }
            wasRunning = running;
            double currentMA = side.getCurrentMA();
            double velocity = side.getVelocity();
            if (running) {
                fullDetector.update(currentMA, side.getRPS(), now);
                peakFilteredMA = Math.max(peakFilteredMA, fullDetector.getFilteredMA());
            }
            KLog.d(TAG, () -> "t=" + now + " side=" + side.getSide() + " running=" + running + " power=" + side.getPower()
                    + " currentMA=" + currentMA + " velocity=" + velocity
                    + " filteredMA=" + fullDetector.getFilteredMA() + " balls=" + fullDetector.estimateBalls()
                    + " rps=" + side.getRPS() + " full=" + fullDetector.isFull());

            double filteredMA = fullDetector.getFilteredMA();
            double rps = side.getRPS();
            String detector = !running ? "idle (hold RT)"
                    : fullDetector.isIgnoring(now) ? "spin-up ignore"
                    : fullDetector.isFull() ? "FULL (latched, release RT to reset)"
                    : fullDetector.isStalling() ? "stalling " + fullDetector.stallHeldMs(now) + "/" + (long) IntakeConfig.STALL_CONFIRM_MS + "ms"
                    : "watching";
            telemetry.addLine("--- Full detector: " + detector + " ---");
            telemetry.addData("balls (estimate)", fullDetector.estimateBalls());
            telemetry.addData("mA raw / filtered / peak", "%.0f / %.0f / %.0f", currentMA, filteredMA, peakFilteredMA);
            telemetry.addData("ball mA 1/2/3/4", "%.0f / %.0f / %.0f / %.0f",
                    IntakeConfig.BALL_1_MA, IntakeConfig.BALL_2_MA, IntakeConfig.BALL_3_MA, IntakeConfig.BALL_4_MA);
            telemetry.addData("stall mA", "%.0f >= %.0f  %s", filteredMA, IntakeConfig.STALL_CURRENT_MA,
                    filteredMA >= IntakeConfig.STALL_CURRENT_MA ? "YES" : "no");
            telemetry.addData("stall RPS", "%.1f <= %.1f  %s", rps, IntakeConfig.STALL_RPS,
                    rps <= IntakeConfig.STALL_RPS ? "YES" : "no");
            telemetry.addData("confirm / ignore / filter", "%.0fms / %.0fms / %.2f",
                    IntakeConfig.STALL_CONFIRM_MS, IntakeConfig.STALL_IGNORE_MS, IntakeConfig.STALL_FILTER);
            telemetry.addLine("--- Intake ---");
            telemetry.addData("side / isDown / pivot", "%s / %b / %.3f", selected, side.isDown(), side.getPivotPosition());
            telemetry.addData("power", "%.2f", side.getPower());
            telemetry.addData("RPS / feed target / kF est", "%.2f / %.2f / %.5f", rps, IntakeConfig.FEED_RPS, side.estimateKf());
            telemetry.addData("flower down / up", "%b / %b", flowerMech.isDown(), flowerMech.isUp());
            telemetry.addLine("GP2: dpad U/D side A/B | A lower | Y raise | RT run | LT reverse | X feed RPS | B flower");
            // plain numbers so the Dashboard can graph them
            telemetry.addData("currentMA", currentMA);
            telemetry.addData("filteredMA", filteredMA);
            telemetry.addData("rps", rps);
            telemetry.update();
        }
        intake.stop();
        cleanupRobot();
    }
}
