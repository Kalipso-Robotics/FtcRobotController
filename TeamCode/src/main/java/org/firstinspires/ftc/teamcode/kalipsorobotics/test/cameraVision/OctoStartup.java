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
 * BRING-UP. Run this FIRST, before OctoTune, and before a tape measure comes out.
 *
 * This OpMode owns every BOOLEAN in OctoConfig. OctoTune owns every float. Signs and
 * handedness are wiring facts, not calibration, and tuning against a backwards pod produces a
 * NEGATIVE counts/mm, which is not a number you can paste anywhere.
 *
 * ONE SCREEN, THREE GESTURES, ANY ORDER. Each gesture latches its answer when it is clean:
 *
 *   FORWARD     push the robot forward ~100 mm+     ch0 and ch2 must count UP
 *                                                   -> INVERT_X, INVERT_X2
 *   TWIST LEFT  turn it left in place, 45 deg+      IMU heading sign
 *                                                   -> MIRROR_BOARD_FRAME (+ means y-LEFT / CCW+)
 *   PUSH LEFT   slide it left ~100 mm+, no turn     ch1 must count UP if mirror, else DOWN
 *                                                   -> INVERT_Y
 *
 * Mirror comes from the IMU, not the encoders, so a twist settles it without an L-test, and
 * once it is known the Y direction can be asserted outright. Press B between gestures: a twist
 * moves ch0 and ch2 oppositely and would foul the forward gate.
 *
 * Raw counts follow setSingleEncoderDirection at once, so a latch measured under the live
 * direction gives the exact flip needed. The localizer only picks a direction up at its next
 * reset, so nothing is relatched while you edit. A commits everything with ONE recalibration,
 * then CONFIRM runs one short L-test as the end-to-end proof that rotation and translation
 * compose. That is the only check that catches a Y port mix-up.
 *
 * GAMEPAD (every value is editable, a manual edit always wins):
 *   dpad UP/DOWN   move the cursor over INVERT_X, INVERT_X2, INVERT_Y, MIRROR_BOARD_FRAME
 *   dpad LEFT/RIGHT flip the selected value
 *   Y              adopt every latched suggestion
 *   A              commit (recalibrates once, only if a value actually changed), go to CONFIRM
 *   B              zero         X   clear all latches and redo the gestures
 *
 *   adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/RobotLogs ~/
 */
@TeleOp
public class OctoStartup extends LinearOpMode {

    /** ~100 mm of pod travel: a deliberate push, not a nudge. */
    private static final int PUSH_COUNTS = (int) (100 * OctoConfig.CPM_THEORETICAL);
    /** A push must stay this flat, or its counts are contaminated by rotation. */
    private static final double QUIET_HEADING_DEG = 15.0;
    /** Anything recognisably a quarter turn will do; this is a sign test. */
    private static final double TWIST_MIN_DEG = 45.0;

    private static final String[] ROW = {"INVERT_X", "INVERT_X2", "INVERT_Y", "MIRROR_BOARD_FRAME"};

    private enum Check { FIRMWARE, GESTURES, CONFIRM, DONE }

    private final OctoQuad.LocalizerDataBlock loc = new OctoQuad.LocalizerDataBlock();
    private final OctoQuad.EncoderDataBlock   enc = new OctoQuad.EncoderDataBlock();
    private final List<String> results = new ArrayList<>();
    private final ElapsedTime runtime = new ElapsedTime();

    private OctoQuad q;
    private KFileWriter csv;
    private Check check = Check.FIRMWARE;

    // Working values, what the paste block prints. applied* is what the localizer last latched.
    private boolean invX  = OctoConfig.INVERT_X;
    private boolean invY  = OctoConfig.INVERT_Y;
    private boolean invX2 = OctoConfig.INVERT_X2;
    private boolean mirror = OctoConfig.MIRROR_BOARD_FRAME;
    private boolean appliedInvX = invX, appliedInvY = invY, appliedInvX2 = invX2;

    // Latched gesture results. null = not seen yet. Suggestions are absolute targets, so a
    // later manual edit cannot make them stale.
    private Boolean sugX, sugX2, sugMirror;
    private boolean yLatched;
    private int latchedRawY;
    private boolean yInvAtLatch;
    private boolean mirrorEdited;
    private int cursor;

    // Widened to double at the read site: posX_mm and posY_mm are SHORTS, and a %f against a
    // boxed Short throws inside telemetry.update() rather than at the line that caused it.
    private double bx, by, headingDeg;
    private int rawX, rawY, rawX2;
    private boolean dataOk;
    private int badReads;

    private Byte chipId;
    private OctoQuad.FirmwareVersion firmware;

    // ----------------------------------------------------------------- pure decisions

    /** Flip needed so a pod that must count UP does. Measured under the direction `inv`. */
    public static boolean suggestInvert(boolean inv, int raw) {
        return inv ^ (raw < 0);
    }

    /**
     * A left push must count UP when the board is y-LEFT (mirror) and DOWN when it is y-RIGHT.
     * `inv` is the direction the pod was set to when rawY was measured.
     */
    public static boolean suggestInvertY(boolean inv, int rawY, boolean mirror) {
        boolean wrong = (rawY > 0) != mirror;
        return inv ^ wrong;
    }

    /** A left turn reading positive means CCW+ / y-left, which is the mirror of ours. */
    public static boolean suggestMirror(double headingDeg) {
        return headingDeg > 0;
    }

    @Override
    public void runOpMode() {

        q = hardwareMap.get(OctoQuad.class, OctoConfig.HARDWARE_NAME);

        // The one place a full reset belongs: bring-up genuinely wants a clean board.
        // OctoConfig.apply() deliberately does not do this, because it runs at every init.
        q.resetEverything();
        OctoConfig.apply(q);

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        csv = new KFileWriter("OctoStartup", opModeUtilities);
        csv.writeLine("check,t_s,x_mm,y_mm,heading_deg,rawX,rawY,rawX2,invX,invY,invX2,crcOk");

        telemetry.addLine("OCTO BRING-UP -- run this before OctoTune.");
        telemetry.addLine();
        telemetry.addLine("Settles the four booleans in OctoConfig:");
        telemetry.addLine("  INVERT_X, INVERT_X2, INVERT_Y, MIRROR_BOARD_FRAME");
        telemetry.addLine();
        telemetry.addLine("You move the robot by hand. Nothing here drives motors.");
        telemetry.addLine("dpad U/D cursor | dpad L/R flip | Y adopt | A commit | B zero | X clear");
        telemetry.addLine("Every commit or finish autosaves to OctoStartup_LATEST.txt.");
        telemetry.addLine("Press START.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) { csv.close(); return; }

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);
        runtime.reset();

        try {
            while (opModeIsActive()) {
                q.readLocalizerDataAndAllEncoderData(loc, enc);
                dataOk = loc.isDataValid() && enc.isDataValid();
                if (!dataOk) {
                    badReads++;
                } else {
                    bx = loc.posX_mm;
                    by = loc.posY_mm;
                    headingDeg = Math.toDegrees(loc.heading_rad);
                    rawX  = enc.positions[OctoConfig.CH_X];
                    rawY  = enc.positions[OctoConfig.CH_Y];
                    rawX2 = enc.positions[OctoConfig.CH_X2];
                }

                if (check != Check.DONE && check != Check.FIRMWARE) {
                    csv.writeLine(String.format(Locale.US,
                            "%s,%.3f,%.1f,%.1f,%.3f,%d,%d,%d,%b,%b,%b,%b",
                            check, runtime.seconds(), bx, by, headingDeg,
                            rawX, rawY, rawX2, invX, invY, invX2, loc.crcOk));
                }

                if (gamepad1.bWasPressed()) zero();
                if (check == Check.GESTURES) latchGestures();
                render();
                telemetry.update();
            }
        } finally {
            csv.close();
        }
    }

    private void zero() {
        q.setLocalizerPose(0, 0, 0f);
        q.resetAllPositions();
        rawX = rawY = rawX2 = 0;
        bx = by = headingDeg = 0;
    }

    // ------------------------------------------------------------------------ gestures

    /**
     * Each gate rejects cross-contamination: a forward push must not turn, a sideways push must
     * not go forward. Results latch once and hold; X clears them to redo.
     */
    private void latchGestures() {
        boolean flat = Math.abs(headingDeg) < QUIET_HEADING_DEG;

        if (flat && Math.abs(rawX) >= PUSH_COUNTS && Math.abs(rawY) < Math.abs(rawX) / 3) {
            if (sugX == null) {
                sugX = suggestInvert(invX, rawX);
                csv.writeLine("# LATCH forward ch0 " + rawX + " -> INVERT_X " + sugX);
            }
            if (sugX2 == null && Math.abs(rawX2) >= PUSH_COUNTS / 2) {
                sugX2 = suggestInvert(invX2, rawX2);
                csv.writeLine("# LATCH forward ch2 " + rawX2 + " -> INVERT_X2 " + sugX2);
            }
        }

        if (sugMirror == null && Math.abs(headingDeg) > TWIST_MIN_DEG) {
            sugMirror = suggestMirror(headingDeg);
            csv.writeLine(String.format(Locale.US, "# LATCH twist heading %.1f -> mirror %b",
                    headingDeg, sugMirror));
        }

        if (!yLatched && flat && Math.abs(rawY) >= PUSH_COUNTS
                && Math.abs(rawX) < Math.abs(rawY) / 3) {
            yLatched = true;
            latchedRawY = rawY;
            yInvAtLatch = invY;
            csv.writeLine("# LATCH left push ch1 " + rawY + " (INVERT_Y was " + invY + ")");
        }
    }

    private boolean mirrorKnown() { return sugMirror != null || mirrorEdited; }

    /** Suggested value for a row, or null if its gesture has not latched (or cannot be read yet). */
    private Boolean suggestion(int row) {
        switch (row) {
            case 0:  return sugX;
            case 1:  return sugX2;
            case 2:  return yLatched && mirrorKnown()
                    ? suggestInvertY(yInvAtLatch, latchedRawY, mirror) : null;
            default: return sugMirror;
        }
    }

    private boolean value(int row) {
        switch (row) {
            case 0:  return invX;
            case 1:  return invX2;
            case 2:  return invY;
            default: return mirror;
        }
    }

    /**
     * Live on the board so the raw counts show the effect at once, with no recalibration.
     * The localizer itself only picks a direction up at the reset A performs.
     */
    private void setValue(int row, boolean v) {
        switch (row) {
            case 0:  invX  = v; OctoConfig.setEncoderDirection(q, OctoConfig.CH_X,  v); break;
            case 1:  invX2 = v; OctoConfig.setEncoderDirection(q, OctoConfig.CH_X2, v); break;
            case 2:  invY  = v; OctoConfig.setEncoderDirection(q, OctoConfig.CH_Y,  v); break;
            default: mirror = v; mirrorEdited = true; break;
        }
    }

    private void handleEditing() {
        if (gamepad1.dpadUpWasPressed())   cursor = (cursor + ROW.length - 1) % ROW.length;
        if (gamepad1.dpadDownWasPressed()) cursor = (cursor + 1) % ROW.length;
        if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed()) {
            setValue(cursor, !value(cursor));
        }
        if (gamepad1.yWasPressed()) {
            // Mirror first: the Y suggestion reads it.
            for (int row : new int[] {3, 0, 1, 2}) {
                Boolean s = suggestion(row);
                if (s != null) setValue(row, s);
            }
        }
        if (gamepad1.xWasPressed()) {
            sugX = sugX2 = sugMirror = null;
            yLatched = mirrorEdited = false;
        }
    }

    private boolean dirty() {
        return invX != appliedInvX || invY != appliedInvY || invX2 != appliedInvX2;
    }

    /** The one relatch. Skipped when nothing changed: the localizer already has these values. */
    private void commit() {
        boolean relatched = dirty();
        if (relatched) {
            if (!OctoConfig.calibrateImu(q, this)) {
                results.add("commit    IMU CALIBRATION FAILED -- nothing below is meaningful");
                enterCheck(Check.DONE);
                return;
            }
            appliedInvX = invX; appliedInvY = invY; appliedInvX2 = invX2;
        }
        results.add(String.format(Locale.US,
                "gestures INVERT_X %b  INVERT_X2 %b  INVERT_Y %b  MIRROR %b%s",
                invX, invX2, invY, mirror, relatched ? "  (relatched)" : ""));
        enterCheck(Check.CONFIRM);
    }

    // ------------------------------------------------------------------------ rendering

    private void render() {
        switch (check) {
            case FIRMWARE: renderFirmware(); break;
            case GESTURES: renderGestures(); break;
            case CONFIRM:  renderConfirm();  break;
            case DONE:     renderDone();     break;
        }
    }

    /**
     * Refuse to go further if this is not an OctoQuad running localizer-capable firmware.
     * There is no MK2 detection method in the SDK, so chip id plus firmware major is the
     * proxy: the localizer API only exists on MK2 firmware, major 3 and up.
     */
    private void renderFirmware() {
        // Read once, not every loop: these are I2C round trips and the values cannot change.
        if (chipId == null) {
            chipId = q.getChipId();
            firmware = q.getFirmwareVersion();
        }
        byte chip = chipId;
        OctoQuad.FirmwareVersion fw = firmware;
        boolean chipOk = chip == OctoQuad.OCTOQUAD_CHIP_ID;
        boolean fwOk = fw.maj >= OctoQuad.SUPPORTED_FW_VERSION_MAJ;

        telemetry.addLine("0/2 BOARD IDENTITY");
        telemetry.addData("chip id",  "0x%02X  (expect 0x%02X)  %s",
                chip, OctoQuad.OCTOQUAD_CHIP_ID, chipOk ? "OK" : "WRONG");
        telemetry.addData("firmware", "%d.%d.%d  (need major >= %d)  %s",
                fw.maj, fw.min, fw.eng, OctoQuad.SUPPORTED_FW_VERSION_MAJ, fwOk ? "OK" : "TOO OLD");
        telemetry.addLine();

        if (!chipOk) {
            telemetry.addLine("*** Not an OctoQuad on this I2C port. Check the config name");
            telemetry.addData("    ", "'%s' and the wiring before anything else.",
                    OctoConfig.HARDWARE_NAME);
        } else if (!fwOk) {
            telemetry.addLine("*** Firmware too old for the onboard localizer. The whole");
            telemetry.addLine("    design depends on it. Update the board, do not work around.");
        } else {
            telemetry.addLine("Board looks right. Press A to calibrate the IMU.");
            telemetry.addLine("The robot must be COMPLETELY STILL while it calibrates:");
            telemetry.addLine("a bump gives a bad heading zero with no error and no symptom");
            telemetry.addLine("until a spin test fails much later.");
        }
        telemetry.addLine("X skips the identity check but still calibrates the IMU --");
        telemetry.addLine("that part can't be skipped, everything below depends on it.");

        boolean skip = gamepad1.xWasPressed();
        if ((gamepad1.aWasPressed() && chipOk && fwOk) || skip) {
            results.add(skip
                    ? "0 board   SKIPPED identity check"
                    : String.format(Locale.US, "0 board   chip 0x%02X  fw %d.%d.%d  OK",
                            chip, fw.maj, fw.min, fw.eng));
            if (OctoConfig.calibrateImu(q, this)) {
                enterCheck(Check.GESTURES);
            } else {
                results.add("0 board   IMU CALIBRATION FAILED -- nothing below is meaningful");
                enterCheck(Check.DONE);
            }
        }
    }

    private void renderGestures() {
        telemetry.addLine("1/2 GESTURES -- any order, B to zero between them");
        telemetry.addLine("  FORWARD     push it straight forward ~100 mm+");
        telemetry.addLine("  TWIST LEFT  turn it left in place, 45 deg+");
        telemetry.addLine("  PUSH LEFT   slide it left ~100 mm+, no turn");
        telemetry.addLine();
        telemetry.addLine(String.format(Locale.US,
                "live  ch0 %7d  ch2 %7d  ch1 %7d  heading %7.1f", rawX, rawX2, rawY, headingDeg));
        telemetry.addLine(String.format(Locale.US,
                "FORWARD     %s", sugX == null && sugX2 == null ? "--"
                        : "ch0 " + (sugX == null ? "--" : "latched")
                        + "  ch2 " + (sugX2 == null ? "--" : "latched")));
        telemetry.addLine("TWIST LEFT  " + (sugMirror == null ? "--"
                : "latched, board is " + (sugMirror ? "y-LEFT / CCW+" : "y-RIGHT / CW+")));
        telemetry.addLine("PUSH LEFT   " + (!yLatched ? "--"
                : mirrorKnown() ? "latched" : "latched, needs TWIST (or edit MIRROR) to read"));
        telemetry.addLine();

        telemetry.addLine("   value                now    suggested");
        for (int row = 0; row < ROW.length; row++) {
            Boolean s = suggestion(row);
            String sug = s == null ? "--" : (s == value(row) ? "ok" : s.toString() + " <- Y");
            telemetry.addLine(String.format(Locale.US, "%s %-20s %-6b %s",
                    row == cursor ? ">" : " ", ROW[row], value(row), sug));
        }
        telemetry.addLine();
        telemetry.addLine("dpad U/D cursor | dpad L/R flip | Y adopt | X clear | A commit");
        telemetry.addLine(dirty()
                ? "(A recalibrates once: hold the robot STILL for ~2 s)"
                : "(no direction changed: A will not recalibrate)");
        if (!dataOk) telemetry.addData("*** BAD READ", "crc/status invalid, %d so far", badReads);

        handleEditing();
        if (gamepad1.aWasPressed()) commit();
    }

    /**
     * End-to-end proof, not discovery. The only check that combines rotation with translation,
     * which is the one place a handedness error or a Y port mix-up can hide. A self-consistent
     * board reports y and heading with the SAME sign, and a left turn must read as the mirror
     * decision says. Anything recognisably a quarter turn left will do.
     */
    private void renderConfirm() {
        boolean turned = Math.abs(headingDeg) > TWIST_MIN_DEG;
        boolean travelled = Math.abs(by) > 100;
        boolean sameSign = Math.signum(by) == Math.signum(headingDeg);
        boolean boardMirror = headingDeg > 0;

        String reason = null;
        if (!turned) {
            reason = String.format(Locale.US,
                    "heading is only %.1f deg. Turn about a quarter turn LEFT.", headingDeg);
        } else if (!travelled) {
            reason = String.format(Locale.US,
                    "y is only %.0f mm. After turning, push it forward a foot or so.", by);
        } else if (!sameSign) {
            reason = "y and heading DISAGREE in sign. Press Y to go back and check INVERT_Y "
                    + "(or you turned RIGHT instead of LEFT).";
        } else if (boardMirror != mirror) {
            reason = "a left turn reads " + (boardMirror ? "+" : "-") + " but MIRROR_BOARD_FRAME is "
                    + mirror + ". Press Y to go back and flip it.";
        }

        telemetry.addLine("2/2 CONFIRM -- L-test after the relatch");
        telemetry.addLine("1. B to zero  2. turn about 90 deg LEFT  3. push FORWARD ~1 ft");
        telemetry.addLine("Accuracy does not matter, only signs.");
        telemetry.addLine();
        telemetry.addLine(String.format(Locale.US, "heading %9.2f deg  %s", headingDeg, verdict(turned)));
        telemetry.addLine(String.format(Locale.US, "y       %9.1f mm   %s", by, verdict(travelled)));
        telemetry.addLine(String.format(Locale.US, "x       %9.1f mm   (near 0 if you did not drift)", bx));
        telemetry.addLine("signs   " + (sameSign && turned && travelled
                ? "AGREE -- fusion is self-consistent" : "disagree or move too small"));
        telemetry.addLine();
        OctoConfig.renderAcceptBlock(telemetry, reason);
        telemetry.addLine("Y back to gestures | X skip this confirm");
        if (!dataOk) telemetry.addData("*** BAD READ", "crc/status invalid, %d so far", badReads);

        if (gamepad1.yWasPressed()) {
            enterCheck(Check.GESTURES);
            return;
        }
        boolean skip = gamepad1.xWasPressed();
        if (gamepad1.aWasPressed() || skip) {
            if (reason == null || skip) {
                results.add(reason == null
                        ? String.format(Locale.US, "confirm   h %+.0f  y %+.0f  OK", headingDeg, by)
                        : "confirm   SKIPPED");
                for (String s : pasteBlock()) csv.writeLine(s);
                flush();
                enterCheck(Check.DONE);
            } else {
                shout(reason);
            }
        }
    }

    private void renderDone() {
        telemetry.addLine("=== BRING-UP RESULTS ===");
        for (String r : results) telemetry.addLine(r);
        telemetry.addLine();
        for (String s : pasteBlock()) telemetry.addLine(s);
        telemetry.addLine();
        telemetry.addLine("Paste into OctoConfig, rebuild, THEN run OctoTune.");
        telemetry.addData("bad reads", "%d", badReads);
        telemetry.addLine("CSV: " + csv.getPath());
        telemetry.addLine("Autosaved: OctoStartup_LATEST.txt (same folder)");
    }

    // ------------------------------------------------------------------------- helpers

    private void shout(String reason) {
        telemetry.addLine();
        telemetry.addLine("*** NOT ACCEPTED ***");
        telemetry.addLine(reason);
        telemetry.update();
        csv.writeLine("# BLOCKED " + check + ": " + reason);
        sleep(600);
    }

    private static String verdict(boolean ok) { return ok ? "OK" : "--"; }

    private void enterCheck(Check c) {
        check = c;
        csv.writeLine("# --- check " + c);
        flush();
        writeLatest();
        if (c != Check.DONE) zero();
    }

    /** Flush at every checkpoint so a mid-run stop cannot take the finished checks with it. */
    private void flush() {
        try {
            csv.flush();
        } catch (java.io.IOException e) {
            telemetry.addLine("CSV flush failed: " + e.getMessage());
        }
    }

    private void writeLatest() {
        OctoConfig.writeLatestSnapshot(csv.getPath(), "OctoStartup_LATEST.txt", telemetry,
                results, pasteBlock());
    }

    private String[] pasteBlock() {
        String today = new java.text.SimpleDateFormat("yyyy-MM-dd", Locale.US)
                .format(new java.util.Date());
        return new String[] {
                "// bring-up " + today + " via OctoStartup",
                String.format(Locale.US, "INVERT_X           = %b;", invX),
                String.format(Locale.US, "INVERT_Y           = %b;", invY),
                String.format(Locale.US, "INVERT_X2          = %b;", invX2),
                String.format(Locale.US, "MIRROR_BOARD_FRAME = %b;", mirror),
        };
    }
}
