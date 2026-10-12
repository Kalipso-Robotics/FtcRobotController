package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.beambreak.BeamBreakCountAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.BeamBreakConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakArray;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakSensor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KGamePad;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import java.util.List;

/**
 * Bring-up and tuning OpMode for the four beam break sensors.
 *
 * <h3>Setting the thresholds</h3>
 * Run it with the robot empty, then push a ball into each position in turn. Each sensor row shows
 * the lowest and highest voltage it has seen and a suggested threshold at the midpoint of that
 * window. Once you have swept every position, copy the suggested values into
 * {@link BeamBreakConfig} - or type them straight into the FTC Dashboard, since the config fields
 * are live-tunable, and confirm here before committing them.
 *
 * If a sensor's min and max are nearly equal after you have blocked it, the beam is not actually
 * being broken (check alignment) or BLOCKED_IS_LOW_VOLTAGE is set wrong.
 *
 * <h3>Tuning the debounce</h3>
 * Park balls in the robot and drive it around. The "glitch" column counts raw flips that reversed
 * before finishing their debounce - those are the pickleball holes being rejected, and seeing it
 * climb while the filtered state stays put is the filter doing its job. If the COUNT itself dips
 * while balls sit still, CLEAR_DEBOUNCE_MS is too short.
 *
 * <h3>Checking the plausibility pass</h3>
 * The occupancy string reads A-to-B: {@code X} measured blocked, {@code +} a clear reading the
 * plausibility pass decided must be a ball, {@code .} clear. "measured" is the uncorrected count
 * and "count" is the corrected one; a gap between them means the pass is firing. Press B to
 * toggle correction off and compare.
 *
 * Expect "impossible, but still settling" to flash past while a ball moves through the robot -
 * that is the settle gate holding the correction back until the balls stop, which is what keeps a
 * ball in transit from being counted twice.
 *
 * <h3>Checking the SharedData hand-off</h3>
 * The headline count here is read back out of SharedData, not off the array, so if the publish
 * path breaks this OpMode goes quiet with it. The "SharedData" row shows the snapshot's age -
 * it should sit at a loop time or two, and anything larger means the publisher is not being
 * updated.
 *
 * <h3>Controls</h3>
 * A - reset filters and the min/max voltage windows.
 * B - toggle the plausibility correction.
 */
@TeleOp(name = "Beam Break Test", group = "test")
public class BeamBreakTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        // Statics outlive an OpMode, so drop whatever the last run published before any
        // subscriber can read last run's ball count as if it were this run's.
        SharedData.resetBeamBreakData();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        opModeUtilities.setAllHubs(allHubs);

        BeamBreakCountAction beamBreakCount = new BeamBreakCountAction(opModeUtilities);
        beamBreakCount.setName("beamBreakCount");
        BeamBreakArray array = beamBreakCount.getArray();

        KGamePad kGamePad1 = new KGamePad(gamepad1);

        telemetry.addLine("Beam break bring-up. Push a ball into each position to sweep voltages.");
        telemetry.update();

        waitForStart();

        long loopCount = 0;
        long lastLoopMs = System.currentTimeMillis();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();

            if (kGamePad1.isButtonAFirstPressed()) {
                beamBreakCount.resetFilters();
            }
            if (kGamePad1.isButtonBFirstPressed()) {
                BeamBreakConfig.ENABLE_PATTERN_CORRECTION = !BeamBreakConfig.ENABLE_PATTERN_CORRECTION;
            }

            beamBreakCount.updateCheckDone();

            long nowMs = System.currentTimeMillis();
            long loopMs = nowMs - lastLoopMs;
            lastLoopMs = nowMs;
            loopCount++;

            // Deliberately read back through SharedData rather than the array, so this OpMode
            // exercises the exact path every subscriber will use.
            BeamBreakData published = SharedData.getBeamBreakData();

            telemetry.addData("COUNT", "%d / %d   %s",
                    published.getCount(), published.getCapacity(), published.getOccupancyString());
            telemetry.addData("measured (uncorrected)", published.getMeasuredCount());
            telemetry.addData("pattern", published.isPatternPlausible()
                    ? "plausible"
                    : published.isSettling()
                            ? "impossible, but still settling - not corrected"
                            : "IMPOSSIBLE - corrected " + published.getCorrectionCount() + " position(s)");
            telemetry.addData("correction", "%s%s",
                    BeamBreakConfig.ENABLE_PATTERN_CORRECTION ? "enabled" : "DISABLED",
                    published.isSettling() ? " (gated: settling)" : "");
            telemetry.addData("SharedData", "%s, age %d ms",
                    published.hasData() ? "publishing" : "NOTHING PUBLISHED YET",
                    published.hasData() ? published.getAgeMs() : 0);
            if (!array.isCountReliable()) {
                telemetry.addData("!! SENSOR FAULT", array.getFaultedPositions());
            }
            telemetry.addLine();

            telemetry.addLine("pos    volts  [min  max] sugg  thr  raw filt  ms  trans glitch");
            for (BeamBreakPosition position : BeamBreakPosition.values()) {
                BeamBreakSensor sensor = array.getSensor(position);
                telemetry.addLine(String.format(
                        "%-6s %5.2f  [%4.2f %4.2f] %4.2f %4.2f  %s  %s %5d %5d %5d",
                        position.shortLabel(),
                        sensor.getLastVoltage(),
                        sensor.getMinVoltageSeen(),
                        sensor.getMaxVoltageSeen(),
                        sensor.getSuggestedThresholdVolts(),
                        sensor.getThresholdVolts(),
                        sensor.isRawBlocked() ? "X" : ".",
                        sensor.isBlocked() ? "X" : (published.isInferredBlocked(position) ? "+" : "."),
                        sensor.getMsInFilteredState(nowMs),
                        sensor.getFilteredTransitionCount(),
                        sensor.getRejectedGlitchCount()));
            }
            telemetry.addLine();

            telemetry.addData("blockedDebounceMs", BeamBreakConfig.BLOCKED_DEBOUNCE_MS);
            telemetry.addData("clearDebounceMs", BeamBreakConfig.CLEAR_DEBOUNCE_MS);
            telemetry.addData("hysteresisV", BeamBreakConfig.HYSTERESIS_V);
            telemetry.addData("correctionSettleMs", BeamBreakConfig.CORRECTION_SETTLE_MS);
            telemetry.addData("blockedIsLowVoltage", BeamBreakConfig.BLOCKED_IS_LOW_VOLTAGE);
            telemetry.addData("loop", "%d ms (%d loops)", loopMs, loopCount);
            telemetry.addLine("A = reset filters/voltage window   B = toggle correction");
            telemetry.update();
        }
    }
}
