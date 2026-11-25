package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeFullDetector;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakArray;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;

/**
 * Hand-driven check of one intake side at a time. Pattern source: decode/calibrations/StopperCalibration (KOpMode skeleton).
 * GP2: dpad up/down = side A/B, A = lower, Y = raise, RT = run, LT = reverse, B = flower toggle,
 * X (hold) = selected side closed loop at IntakeConfig.FEED_RPS (the shot feed; tune it in IntakeTest).
 * Flower interlock stays in Flower/IntakeSide; this only calls their methods and logs the result.
 *
 * Also the IntakeFullDetector tuner: hold RT and feed balls in one at a time; currentMA is graphed on the Dashboard
 * (and logged). Set IntakeFullDetector.EMPTY_CURRENT_MA (running empty) and FULL_CURRENT_MA (4 balls packed) from it.
 *
 * SPIKE (motor-signal ball counting): the log line also carries t + beam-break counts as ground truth, so a logcat
 * pull can score a velocity-dip counter against BeamBreakArray offline. Remove the bb* fields when the spike ends.
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
        IntakeFullDetector fullDetector = new IntakeFullDetector();
        BeamBreakArray beamBreaks = new BeamBreakArray(opModeUtilities); // spike: ground truth
        boolean wasRunning = false;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
            beamBreaks.update(now);
            if (running && !wasRunning) {
                fullDetector.reset(now);
            }
            wasRunning = running;
            double currentMA = side.getCurrentMA();
            double velocity = side.getVelocity();
            if (running) {
                fullDetector.update(currentMA, now);
            }
            KLog.d(TAG, () -> "t=" + now + " side=" + side.getSide() + " running=" + running + " power=" + side.getPower()
                    + " currentMA=" + currentMA + " velocity=" + velocity
                    + " filteredMA=" + fullDetector.getFilteredMA() + " balls=" + fullDetector.estimateBalls()
                    + " full=" + fullDetector.isFull() + " bbCount=" + beamBreaks.getCount()
                    + " bbMeasured=" + beamBreaks.getMeasuredCount() + " bbReliable=" + beamBreaks.isCountReliable()
                    + " bbOcc=" + beamBreaks.getOccupancyString());

            telemetry.addData("side", selected);
            telemetry.addData("pivotPos", "%.3f", side.getPivotPosition());
            telemetry.addData("isDown", side.isDown());
            telemetry.addData("power", "%.2f", side.getPower());
            telemetry.addData("currentMA", currentMA);
            telemetry.addData("filteredMA", fullDetector.getFilteredMA());
            telemetry.addData("velocity", velocity);
            telemetry.addData("RPS / feed target / kF est", "%.2f / %.2f / %.5f", side.getRPS(), IntakeConfig.FEED_RPS, side.estimateKf());
            telemetry.addData("ballEstimate", fullDetector.estimateBalls());
            telemetry.addData("full", fullDetector.isFull());
            telemetry.addData("bbCount / reliable", "%d / %b", beamBreaks.getCount(), beamBreaks.isCountReliable());
            telemetry.addData("flowerDown", flowerMech.isDown());
            telemetry.addData("flowerUp", flowerMech.isUp());
            telemetry.update();
        }
        intake.stop();
        cleanupRobot();
    }
}
