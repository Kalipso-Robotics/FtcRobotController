package org.firstinspires.ftc.teamcode.kalipsorobotics.test.cameraVision;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Measures COUNTS_PER_MM for the X and Y pods, and IMU_HEADING_SCALAR. Two tape-measure
 * pushes and one hand-turned spin, nothing else.
 *
 * Run OctoStartup FIRST. That OpMode owns every BOOLEAN in OctoConfig. A pod wired backwards
 * produces a NEGATIVE counts/mm here, which is not a number you can paste anywhere.
 *
 * WHAT THIS DELIBERATELY DOES NOT DO, and where those numbers come from instead:
 *
 *   TCP_OFFSET_MM_X/Y    tape measure. See the javadoc on those constants in OctoConfig. The
 *                        SDK itself says they need not be accurate. Solving them with a circle
 *                        fit was 200 lines buying precision the board does not use.
 *   TRACK_WIDTH_MM       tape measure between the two parallel pod wheels. Monitor only; the
 *                        board never reads it.
 *   static drift, L-loop these measured nothing that gets pasted anywhere. Drift is a mounting
 *                        problem you see in OctoTest, and out-and-back CANCELS scale error, so
 *                        the loop was never a scale check to begin with.
 *
 * The board consumes exactly five floats (OctoConfig.apply -> setAllLocalizerParameters).
 * This file produces three of them. The other two are ruler work.
 *
 * COUNTS/MM COMES FROM RAW COUNTS, NOT FROM THE REPORTED POSE.
 *
 *     countsPerMM = rawCounts / tapeDistanceMm
 *
 * That is one division against the floor, with no board arithmetic in the path. The old version
 * worked backwards from the reported pose, which is FIELD-frame: it has already been rotated by
 * heading and shifted by the TCP offsets. On a perfectly straight push the two agree exactly,
 * and the moment the robot twists a few degrees they do not. Both are on screen, and a gap
 * between them IS the twist. Believe the raw one.
 *
 * IMU_HEADING_SCALAR is the SDK's own prescribed procedure (javadoc on
 * setLocalizerImuHeadingScalar): scalar = 1.0, rotate the robot 10 full turns by hand against a
 * hard surface, scalar = (rotations * 360) / reportedTotalDeg. This is the one knob that fights
 * position error over a LONG run: error goes roughly as the integral of heading error along the
 * path, so a heading scale that is off by even 1% compounds over a full match in a way a fixed
 * TCP offset or a one-time counts/mm error does not.
 *
 * SPACE: two straight lanes with a hard stop at each end, tape-measured stop to stop, plus a
 * wall or other straightedge to spin the robot against. They do not have to be an L and there
 * is no rotation in the linear stages, so one lane used twice is fine if the robot can be set
 * down facing either way. The calibration cannot be more accurate than that one tape
 * measurement: at 72 in, 1/16 in is 0.09%.
 *
 * ALL MOVEMENT IS BY HAND. Nothing here drives a motor.
 *
 * BUTTONS:  A accept   B zero   X skip this stage (keep its current/seeded value, no push needed)
 *
 * A never silently does nothing. If it will not accept, it says why, on screen. X always works:
 * it is for stages you already trust (e.g. pasted from a previous run) and do not want to redo.
 *
 * Every accepted OR skipped stage overwrites OctoTune_LATEST.txt, next to the timestamped CSV,
 * with the current results and paste block -- autosaved, no need to copy anything off the
 * screen by hand. Nothing here is written to the board or to flash; that file is the output,
 * source is the only source of truth. Paste, rebuild, then run OctoTest.
 *   adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/RobotLogs ~/
 */
@TeleOp(name = "Octo 2 Tune", group = "Calibration")
public class OctoTune extends LinearOpMode {

    private enum Stage { LINEAR_X, LINEAR_Y, HEADING, DONE }

    /**
     * Board frame -> your frame, for y and heading TOGETHER. See OctoConfig.toRobotFrame.
     * Used only for the on-screen cross-check numbers; the counts/mm answer never touches it.
     */
    private static final double MIRROR = OctoConfig.MIRROR_BOARD_FRAME ? -1.0 : 1.0;

    /** Refuse a push shorter than this fraction of the lane. Half a lane is not a typo. */
    private static final double MIN_PUSH_FRACTION = 0.5;

    /** Deviation from CPM_THEORETICAL that earns a loud warning (not a block). Percent. */
    private static final double WARN_DEVIATION_PCT = 5.0;

    private final OctoQuad.LocalizerDataBlock loc = new OctoQuad.LocalizerDataBlock();
    private final OctoQuad.EncoderDataBlock   enc = new OctoQuad.EncoderDataBlock();
    private final List<String> results = new ArrayList<>();
    private final ElapsedTime runtime = new ElapsedTime();

    private OctoQuad q;
    private KFileWriter csv;
    private Stage stage = Stage.LINEAR_X;

    // Working values, seeded from OctoConfig and replaced as pushes are accepted.
    private float cpmX  = OctoConfig.COUNTS_PER_MM_X;
    private float cpmX2 = OctoConfig.COUNTS_PER_MM_X2;
    private float cpmY  = OctoConfig.COUNTS_PER_MM_Y;
    private float imuHeadingScalar = OctoConfig.IMU_HEADING_SCALAR;

    /**
     * Board pose widened to double at the read site.
     *
     * posX_mm and posY_mm are SHORTS. FTC telemetry defers String.format to update(), so a %f
     * against a boxed Short throws IllegalFormatConversionException at update() rather than at
     * the line that caused it. Widening here costs nothing and makes every format string and
     * CSV row below type-safe.
     */
    private double bx, by, headingDeg;
    private boolean dataOk;
    private int badReads;

    // Raw counts are zeroed in software rather than on the board. See zero().
    private int rawXAbs, rawYAbs, rawX2Abs;
    private int rawX0, rawY0, rawX20;

    // HEADING stage: unwrapped total rotation, and how close the raw signal got to the
    // board's own wire range while doing it. See updateHeadingUnwrap().
    private double turnedDeg;
    private double lastRawHeadingRad;
    private boolean headingUnwrapSeeded;
    private double maxAbsHeadingRad;

    @Override
    public void runOpMode() {

        q = hardwareMap.get(OctoQuad.class, OctoConfig.HARDWARE_NAME);
        OctoConfig.apply(q);

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        csv = new KFileWriter("OctoTune", opModeUtilities);
        csv.writeLine(String.format(Locale.US,
                "# seeds cpmX=%.5f cpmX2=%.5f cpmY=%.5f theoretical=%.5f "
                + "podCPR=%.0f podWheelMM=%.1f invX=%b invY=%b invX2=%b mirror=%b",
                cpmX, cpmX2, cpmY, OctoConfig.CPM_THEORETICAL,
                OctoConfig.POD_COUNTS_PER_REV, OctoConfig.POD_WHEEL_MM,
                OctoConfig.INVERT_X, OctoConfig.INVERT_Y, OctoConfig.INVERT_X2,
                OctoConfig.MIRROR_BOARD_FRAME));
        csv.writeLine(String.format(Locale.US,
                "# lanes pushX=%.1f in (%.1f mm)  pushY=%.1f in (%.1f mm)",
                OctoConfig.PUSH_X_IN, OctoConfig.PUSH_X_MM,
                OctoConfig.PUSH_Y_IN, OctoConfig.PUSH_Y_MM));
        csv.writeLine("stage,t_s,rawX,rawY,rawX2,x_mm,y_mm,heading_deg,crcOk");

        telemetry.addLine("OCTO TUNE -- counts/mm only. Run OctoStartup first.");
        telemetry.addData("Lane X", "%.0f in, hard stops both ends", OctoConfig.PUSH_X_IN);
        telemetry.addData("Lane Y", "%.0f in, hard stops both ends", OctoConfig.PUSH_Y_IN);
        telemetry.addLine("Measure each lane stop-to-stop with a tape and make PUSH_X_IN /");
        telemetry.addLine("PUSH_Y_IN in OctoConfig match it. Nothing here is more accurate");
        telemetry.addLine("than that one measurement.");
        telemetry.addLine();
        telemetry.addLine("A accept | B zero | X skip stage (keep current value)");
        telemetry.addLine("Press START.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) { csv.close(); return; }

        // The pose and heading cross-checks need a running localizer. The counts/mm answer does
        // not, but a failed calibration usually means a board that is not talking properly, so
        // it is still worth stopping for.
        if (!OctoConfig.calibrateImu(q, this)) {
            telemetry.addLine("IMU calibration failed. The board is not healthy; fix that first.");
            telemetry.update();
            csv.writeLine("# ABORT imu calibration failed");
            csv.close();
            sleep(4000);
            return;
        }

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);

        runtime.reset();
        primeRead();
        enterStage(Stage.LINEAR_X);

        try {
            while (opModeIsActive()) {
                q.readLocalizerDataAndAllEncoderData(loc, enc);
                dataOk = loc.isDataValid() && enc.isDataValid();

                if (!dataOk) {
                    badReads++;
                } else {
                    bx = loc.posX_mm;
                    by = MIRROR * loc.posY_mm;
                    // Raw, not unwrapped. The linear stages don't rotate, so a heading anywhere
                    // near the wrap point there means the push twisted far past any usable
                    // tolerance and the twist check below has already failed. The HEADING stage
                    // rotates on purpose; it tracks total turn separately via turnedDeg.
                    headingDeg = MIRROR * Math.toDegrees(loc.heading_rad);
                    rawXAbs  = enc.positions[OctoConfig.CH_X];
                    rawYAbs  = enc.positions[OctoConfig.CH_Y];
                    rawX2Abs = enc.positions[OctoConfig.CH_X2];
                    updateHeadingUnwrap(loc.heading_rad);
                }

                if (stage != Stage.DONE) {
                    csv.writeLine(String.format(Locale.US,
                            "%s,%.3f,%d,%d,%d,%.2f,%.2f,%.4f,%b",
                            stage, runtime.seconds(), rawX(), rawY(), rawX2(),
                            bx, by, headingDeg, loc.crcOk));
                }

                if (gamepad1.bWasPressed()) zero();

                switch (stage) {
                    case LINEAR_X: renderLinear(true);  break;
                    case LINEAR_Y: renderLinear(false); break;
                    case HEADING:  renderHeading();      break;
                    case DONE:     renderDone();        break;
                }
                telemetry.update();
            }
        } finally {
            csv.close();
        }
    }

    /**
     * One push, one division.
     *
     * X and Y are measured independently against their OWN lane length and will not come out
     * equal. The two pods sit under different loads and their rubber compresses differently.
     * Sharing one push distance between them would silently scale cpmY by PUSH_X/PUSH_Y and
     * still look plausible.
     *
     * The X push also yields the monitor pod (port 2) scale for free, off the same travel.
     */
    private void renderLinear(boolean isX) {
        int    raw      = isX ? rawX() : rawY();
        double actualMm = isX ? OctoConfig.PUSH_X_MM : OctoConfig.PUSH_Y_MM;
        double actualIn = isX ? OctoConfig.PUSH_X_IN : OctoConfig.PUSH_Y_IN;

        float measured  = (float) (raw / actualMm);
        float measuredX2 = isX ? (float) (rawX2() / actualMm) : 0f;

        // Same quantity via the board's reported pose. Equal to `measured` on a clean push;
        // any gap is twist (or a TCP offset) leaking into the field-frame pose.
        double reported = isX ? bx : by;
        float  viaPose  = (float) ((isX ? cpmX : cpmY) * (reported / actualMm));

        double deviation = Math.abs(measured - OctoConfig.CPM_THEORETICAL)
                / OctoConfig.CPM_THEORETICAL * 100.0;
        double impliedWheelMm = measured <= 0 ? 0
                : OctoConfig.POD_COUNTS_PER_REV / (Math.PI * measured);
        double crossMm = isX ? by : bx;

        // Travel estimated at the SEED scale, only so the "did you actually push it" gate has
        // something to compare against. It cannot use `measured`: that is defined as
        // raw / actualMm, so raw / measured is actualMm identically and the gate would never
        // fire. The seed may be the wrong scale -- that is the whole point of this OpMode --
        // so the threshold is deliberately loose at half a lane.
        double travelAtSeedMm = raw / (isX ? OctoConfig.COUNTS_PER_MM_X
                                           : OctoConfig.COUNTS_PER_MM_Y);

        String axis = isX ? "X" : "Y";
        String reason = null;
        if (raw < 0) {
            reason = "raw counts went NEGATIVE. The " + axis + " pod counts backwards for this "
                    + "push. Stop, run OctoStartup and fix INVERT_" + axis + ". Accepting this "
                    + "would paste a negative counts/mm into OctoConfig.";
        } else if (raw == 0) {
            reason = "no counts at all on the " + axis + " pod. Zero with B at the start stop, "
                    + "then push. If it stays at zero, the pod is not reaching the board.";
        } else if (travelAtSeedMm < actualMm * MIN_PUSH_FRACTION) {
            reason = String.format(Locale.US,
                    "only about %.0f mm of %.0f travelled (estimated at the SEED scale). Push "
                    + "all the way to the far stop. If it IS at the far stop, the seed scale is "
                    + "off by more than 2x, which is a pod spec or wiring problem, not tuning.",
                    travelAtSeedMm, actualMm);
        } else if (isX && rawX2() <= 0) {
            reason = "monitor pod (ch2) counts <= 0 on a forward push. Port 2 is wired or "
                    + "inverted backwards; fix it in OctoStartup.";
        }

        telemetry.addLine(isX ? "1/2 PUSH X -- push the robot FORWARD"
                              : "2/2 PUSH Y -- push the robot RIGHT");
        telemetry.addLine("1. set the robot against the START stop");
        telemetry.addLine("2. press B to zero");
        telemetry.addLine("3. push slowly to the FAR stop, do not twist or lift");
        telemetry.addLine("4. press A to record");
        if (!isX) telemetry.addLine("RIGHT, not left: +Y is out the robot's right side.");
        telemetry.addLine();
        telemetry.addData("tape distance", "%8.1f in  (%.1f mm)", actualIn, actualMm);
        telemetry.addData("raw counts",    "%8d   <- THE measurement", raw);
        telemetry.addData("counts/mm",     "%8.5f  <- THE answer", (double) measured);
        telemetry.addData("X to skip",     "keep seeded %.5f, no push needed", (double) (isX ? cpmX : cpmY));
        telemetry.addLine();
        telemetry.addData("theoretical",   "%8.5f  (%.1f mm wheel, %.0f CPR)",
                (double) OctoConfig.CPM_THEORETICAL,
                (double) OctoConfig.POD_WHEEL_MM, (double) OctoConfig.POD_COUNTS_PER_REV);
        telemetry.addData("off theoretical", "%8.2f %%", deviation);
        telemetry.addData("implies wheel", "%8.1f mm diameter  <- CHECK WITH CALIPERS",
                impliedWheelMm);
        if (deviation > WARN_DEVIATION_PCT) {
            telemetry.addLine("*** That is FAR too big to be rubber compression. Either the pod");
            telemetry.addLine("*** wheel is not the diameter POD_WHEEL_MM claims, or the pod is");
            telemetry.addLine("*** slipping, or the tape distance is wrong. A is still allowed:");
            telemetry.addLine("*** a measurement is data. But find out which it is first.");
        }
        telemetry.addLine();
        telemetry.addData("via pose",   "%8.5f  cross-check, should match", (double) viaPose);
        telemetry.addData("cross-axis", "%8.1f mm  should be ~0 (twist check)", crossMm);
        telemetry.addData("heading",    "%8.2f deg should be ~0 (twist check)", headingDeg);
        if (isX) {
            telemetry.addData("counts/mm X2", "%8.5f  (monitor pod, port 2)", (double) measuredX2);
        }
        renderHealth();
        renderBlock(reason);

        if (gamepad1.aWasPressed()) {
            if (reason != null) { shout(reason); return; }
            if (isX) {
                cpmX  = measured;
                cpmX2 = measuredX2;
            } else {
                cpmY = measured;
            }
            results.add(String.format(Locale.US,
                    "push %s  %6d counts / %.1f mm = %.5f counts/mm  (%.1f%% off theoretical)",
                    axis, raw, actualMm, measured, deviation));
            csv.writeLine(String.format(Locale.US,
                    "# RESULT push %s raw %d actualMm %.1f cpm %.5f viaPose %.5f dev %.2f%% "
                    + "impliedWheelMm %.2f crossMm %.1f headingDeg %.2f",
                    axis, raw, actualMm, measured, viaPose, deviation,
                    impliedWheelMm, crossMm, headingDeg));
            accept(isX ? Stage.LINEAR_Y : Stage.HEADING);
        } else if (gamepad1.xWasPressed()) {
            results.add(String.format(Locale.US,
                    "push %s  SKIPPED -- kept %.5f counts/mm", axis, (double) (isX ? cpmX : cpmY)));
            csv.writeLine("# SKIPPED " + stage + " -- kept seed value");
            accept(isX ? Stage.LINEAR_Y : Stage.HEADING);
        }
    }

    /**
     * SDK's own prescribed procedure for setLocalizerImuHeadingScalar: seed at 1.0, spin the
     * robot a known number of full turns by hand, scalar = expected / measured.
     *
     * Uses turnedDeg (summed absolute unwrapped delta), not the signed net rotation, so small
     * wobble while hand-turning does not cancel out and hide a real gap.
     */
    private void renderHeading() {
        double targetDeg = OctoConfig.SPIN_ROTATIONS * 360.0;
        double measuredScalar = turnedDeg <= 0 ? 0 : targetDeg / turnedDeg;

        String reason = null;
        if (turnedDeg <= 0) {
            reason = "no rotation recorded yet. Zero with B against the wall, then spin.";
        } else if (Math.abs(turnedDeg - targetDeg) / targetDeg > 0.20) {
            reason = String.format(Locale.US,
                    "only %.0f deg of the expected %.0f deg recorded -- that is a miscounted "
                    + "turn, not an IMU error. B to zero and recount to exactly %d full turns, "
                    + "or press X to skip and keep the current scalar.",
                    turnedDeg, targetDeg, OctoConfig.SPIN_ROTATIONS);
        } else if (measuredScalar < 0.9 || measuredScalar > 1.1) {
            reason = String.format(Locale.US,
                    "scalar %.4f is outside 0.9-1.1. That is not a calibration bump, it is a "
                    + "broken heading axis choice -- check getLocalizerHeadingAxisChoice() in "
                    + "OctoStartup before accepting.",
                    measuredScalar);
        } else if (maxAbsHeadingRad > 3.2) {
            reason = String.format(Locale.US,
                    "raw heading reached %.2f rad mid-spin, more than a straight-line assumption "
                    + "should ever need. Whether the board wraps at +/-pi or at its +/-6.55 rad "
                    + "wire range is undocumented, and this unwrap only holds if it never crosses "
                    + "that wrap point between reads. B to zero and redo more slowly, watching "
                    + "for a jump, or press X to skip and keep the current scalar.",
                    maxAbsHeadingRad);
        }

        telemetry.addLine("3/3 HEADING -- spin " + OctoConfig.SPIN_ROTATIONS + " FULL turns");
        telemetry.addLine("1. set the robot against a wall or straightedge");
        telemetry.addLine("2. press B to zero");
        telemetry.addLine("3. spin " + OctoConfig.SPIN_ROTATIONS
                + " full turns, same direction, back against the same wall");
        telemetry.addLine("4. press A to record");
        telemetry.addLine();
        telemetry.addData("turned",   "%8.1f deg  <- THE measurement", turnedDeg);
        telemetry.addData("expected", "%8.1f deg  (%d x 360)", targetDeg, OctoConfig.SPIN_ROTATIONS);
        telemetry.addData("scalar",   "%8.4f  <- THE answer", measuredScalar);
        telemetry.addData("X to skip", "keep seeded %.4f, no spin needed", (double) imuHeadingScalar);
        telemetry.addLine();
        telemetry.addData("max |raw heading|", "%6.2f rad  (wire range +/-6.55)", maxAbsHeadingRad);
        renderHealth();
        renderBlock(reason);

        if (gamepad1.aWasPressed()) {
            if (reason != null) { shout(reason); return; }
            imuHeadingScalar = (float) measuredScalar;
            results.add(String.format(Locale.US,
                    "spin %d turns  %7.1f / %7.1f deg expected = %.4f scalar",
                    OctoConfig.SPIN_ROTATIONS, turnedDeg, targetDeg, measuredScalar));
            csv.writeLine(String.format(Locale.US,
                    "# RESULT heading turnedDeg %.1f expectedDeg %.1f scalar %.4f "
                    + "maxAbsHeadingRad %.2f",
                    turnedDeg, targetDeg, measuredScalar, maxAbsHeadingRad));
            accept(Stage.DONE);
        } else if (gamepad1.xWasPressed()) {
            results.add(String.format(Locale.US,
                    "spin SKIPPED -- kept %.4f scalar", (double) imuHeadingScalar));
            csv.writeLine("# SKIPPED " + stage + " -- kept seed value");
            accept(Stage.DONE);
        }
    }

    private void renderDone() {
        telemetry.addLine("=== RESULTS ===");
        for (String r : results) telemetry.addLine(r);
        telemetry.addLine();
        for (String s : pasteBlock()) telemetry.addLine(s);
        telemetry.addLine();
        telemetry.addLine("Paste into OctoConfig. The other two board floats are tape work:");
        telemetry.addLine("  TCP_OFFSET_MM_X/Y  where the two pod lines cross, to robot centre");
        telemetry.addLine("Then rebuild and run OctoTest.");
        telemetry.addData("bad reads", "%d", badReads);
        telemetry.addLine("CSV: " + csv.getPath());
        telemetry.addLine("Autosaved: OctoTune_LATEST.txt (same folder)");
    }

    // ------------------------------------------------------------------------ helpers

    private void renderHealth() {
        if (!dataOk || badReads > 0) {
            telemetry.addData("health", "bad reads %d  %s",
                    badReads, dataOk ? "" : "<- READ INVALID RIGHT NOW");
        }
    }

    private void renderBlock(String reason) {
        OctoConfig.renderAcceptBlock(telemetry, reason);
    }

    private void shout(String reason) {
        telemetry.addLine();
        telemetry.addLine("*** NOT ACCEPTED ***");
        telemetry.addLine(reason);
        telemetry.update();
        csv.writeLine("# BLOCKED " + stage + ": " + reason);
        flush();
        sleep(800);
    }

    private int rawX()  { return rawXAbs  - rawX0;  }
    private int rawY()  { return rawYAbs  - rawY0;  }
    private int rawX2() { return rawX2Abs - rawX20; }

    /**
     * One read before the first zero(), so the software count offsets are taken against real
     * hardware values rather than against the zeros the block starts life holding.
     *
     * OctoConfig.apply() does not call resetEverything(), so the board arrives here still
     * holding whatever counts the previous OpMode left on it. Zeroing against an unread block
     * would leave the offsets at 0 and the raw accessors returning absolute counts.
     */
    private void primeRead() {
        q.readLocalizerDataAndAllEncoderData(loc, enc);
        if (enc.isDataValid()) {
            rawXAbs  = enc.positions[OctoConfig.CH_X];
            rawYAbs  = enc.positions[OctoConfig.CH_Y];
            rawX2Abs = enc.positions[OctoConfig.CH_X2];
        }
    }

    /**
     * Zero the pose, and zero the raw counts in SOFTWARE.
     *
     * Not resetAllPositions(): that and setLocalizerPose() are not atomic, and the board is
     * integrating at 1.92 kHz between them, so whether clearing the count registers injects a
     * garbage delta into the localizer is undocumented. Subtracting an offset costs three lines
     * and removes the question.
     */
    private void zero() {
        q.setLocalizerPose(0, 0, 0f);
        rawX0  = rawXAbs;
        rawY0  = rawYAbs;
        rawX20 = rawX2Abs;
        bx = by = headingDeg = 0;
        turnedDeg = 0;
        headingUnwrapSeeded = false;
        maxAbsHeadingRad = 0;
    }

    /**
     * Unwraps heading the same way OctoTest does, via OctoConfig.wrapDeltaRad. Whether the
     * board's heading_rad wraps at +/-pi or at its own +/-6.5535 rad wire range (int16 / 5000)
     * is undocumented, so summing bounded per-loop deltas is correct either way -- provided no
     * single loop iteration crosses whichever wrap point is real. maxAbsHeadingRad exists to
     * flag that possibility rather than silently trust a spin that got close to it.
     */
    private void updateHeadingUnwrap(double rawHeadingRad) {
        maxAbsHeadingRad = Math.max(maxAbsHeadingRad, Math.abs(rawHeadingRad));
        if (!headingUnwrapSeeded) {
            lastRawHeadingRad = rawHeadingRad;
            headingUnwrapSeeded = true;
            return;
        }
        double d = OctoConfig.wrapDeltaRad(rawHeadingRad, lastRawHeadingRad);
        lastRawHeadingRad = rawHeadingRad;
        turnedDeg += Math.abs(Math.toDegrees(d));
    }

    private void enterStage(Stage s) {
        stage = s;
        zero();
        csv.writeLine("# --- stage " + s);
    }

    /**
     * Checkpoint to disk on the way to the next push. The paste block is written after EVERY
     * accepted push, not just at the end: getting stuck on the second one used to mean the
     * first one's number existed only in memory and died with the OpMode.
     */
    private void accept(Stage next) {
        for (String s : pasteBlock()) csv.writeLine(s);
        flush();
        writeLatest();
        enterStage(next);
    }

    private void flush() {
        try {
            csv.flush();
        } catch (java.io.IOException e) {
            telemetry.addLine("CSV flush failed: " + e.getMessage());
        }
    }

    private void writeLatest() {
        OctoConfig.writeLatestSnapshot(csv.getPath(), "OctoTune_LATEST.txt", telemetry,
                results, pasteBlock());
    }

    private String[] pasteBlock() {
        String today = new java.text.SimpleDateFormat("yyyy-MM-dd", Locale.US)
                .format(new java.util.Date());
        return new String[] {
                "// measured " + today + " via OctoTune, through " + stage,
                String.format(Locale.US, "COUNTS_PER_MM_X    = %.5ff;", cpmX),
                String.format(Locale.US, "COUNTS_PER_MM_X2   = %.5ff;", cpmX2),
                String.format(Locale.US, "COUNTS_PER_MM_Y    = %.5ff;", cpmY),
                String.format(Locale.US, "IMU_HEADING_SCALAR = %.4ff;", imuHeadingScalar),
        };
    }
}
