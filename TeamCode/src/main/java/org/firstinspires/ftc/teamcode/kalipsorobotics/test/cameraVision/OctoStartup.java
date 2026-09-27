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
 * handedness are wiring facts, not calibration, and they are cheap to get wrong in a way
 * that silently wastes an entire tuning session: tuning against a backwards pod produces
 * a negative counts/mm, which is not a number you can paste anywhere.
 *
 * It replaces a file that had become a verbatim copy of OctoTune. Because of that, nothing
 * in the repo ever set INVERT_X/Y/X2, which is why pushing the robot forward could report a
 * NEGATIVE x and why the old stage 5 refused to accept.
 *
 * Every direction here is toggled LIVE on the board with setSingleEncoderDirection. No
 * edit-rebuild-redeploy cycle to test a sign. That is the whole point of the tool.
 *
 * The three checks:
 *   1 PUSH FORWARD   both parallel pods (ch0, ch2) must count UP, and pose x must go POSITIVE
 *   2 PUSH SIDEWAYS  ch1 must respond at all (alive, plugged into port 1)
 *   3 L-TEST         turn ~90 left and push forward: settles INVERT_Y and MIRROR_BOARD_FRAME
 *
 * Why check 2 does NOT assert a direction for Y:
 *
 * The board fuses X and Y into a pose using its own frame, and that frame is undocumented.
 * Both candidate frames put +X out the front, so check 1 can assert a direction safely. They
 * disagree about Y. If this check demanded "pushing right reads +Y" and the board is natively
 * y-left, the operator would satisfy it by inverting the Y encoder, which is precisely the
 * corruption the warning block in OctoConfig exists to prevent: it passes every straight-line
 * test and fails the moment heading leaves zero.
 *
 * So INVERT_Y and MIRROR_BOARD_FRAME are resolved together, by the L-test. Of the four
 * combinations only two are self-consistent, and a correct fusion reports y and heading with
 * the SAME sign. Opposite signs mean the fusion is corrupted: flip INVERT_Y and redo. One
 * button, one retry, guaranteed to resolve.
 *
 * The legacy-odometry cross-check (does localization/Odometry call the same direction +Y?)
 * lives in OctoTest, which already has the drivetrain instantiated for driving.
 *
 *   adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/RobotLogs ~/
 */
@TeleOp
public class OctoStartup extends LinearOpMode {

    /** Enough counts to be a deliberate push rather than a nudge: ~100mm at the seed scale. */
    private static final int MIN_PUSH_COUNTS = 2000;
    /** The L-test is a sign test. Anything recognisably a quarter turn will do. */
    private static final double LTEST_MIN_HEADING_DEG = 45.0;

    private enum Check { FIRMWARE, PUSH_FORWARD, PUSH_SIDEWAYS, LTEST, DONE }

    private final OctoQuad.LocalizerDataBlock loc = new OctoQuad.LocalizerDataBlock();
    private final OctoQuad.EncoderDataBlock   enc = new OctoQuad.EncoderDataBlock();
    private final List<String> results = new ArrayList<>();
    private final ElapsedTime runtime = new ElapsedTime();

    private OctoQuad q;
    private KFileWriter csv;
    private Check check = Check.FIRMWARE;

    // Live working values. These are what the paste block prints.
    private boolean invX  = OctoConfig.INVERT_X;
    private boolean invY  = OctoConfig.INVERT_Y;
    private boolean invX2 = OctoConfig.INVERT_X2;
    private boolean mirror = OctoConfig.MIRROR_BOARD_FRAME;

    // Widened to double at the read site: posX_mm and posY_mm are SHORTS, and a %f against a
    // boxed Short throws inside telemetry.update() rather than at the line that caused it.
    private double bx, by, headingDeg;
    private int rawX, rawY, rawX2;
    private boolean dataOk;
    private int badReads;

    private Byte chipId;
    private OctoQuad.FirmwareVersion firmware;

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
        telemetry.addLine("  INVERT_X, INVERT_Y, INVERT_X2, MIRROR_BOARD_FRAME");
        telemetry.addLine();
        telemetry.addLine("You push the robot by hand. Nothing here drives motors.");
        telemetry.addLine("A accept | B zero | X skip this check (keep current value)");
        telemetry.addLine("Every accept or skip autosaves to OctoStartup_LATEST.txt.");
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
                handleDirectionToggles();
                render();
                telemetry.update();
            }
        } finally {
            csv.close();
        }
    }

    // ---------------------------------------------------------------- direction toggles

    /**
     * Live on the board, not in source. A sign is then a button press instead of an edit,
     * a rebuild, a redeploy and a walk back to the field.
     *
     * *** A direction change does NOT reach the localizer until the localizer is reset. ***
     *
     * setSingleEncoderDirection flips the reported COUNTS immediately, but the localizer keeps
     * using whatever direction was latched at the last resetLocalizerAndCalibrateIMU(). The SDK
     * sample says so in one line above its direction calls: "these parameter changes will not
     * take effect until the localizer is reset". Without the reset below, the toggle makes ch0
     * and ch2 count up while pose x stays negative, and no combination of the three booleans
     * ever fixes it -- the pose is not listening to any of them.
     *
     * Toggling therefore re-latches and re-zeros: the counts either side of a direction change
     * are not comparable, and the IMU recalibration wants the robot still anyway, which it is
     * while someone is pressing dpad.
     */
    private void handleDirectionToggles() {
        boolean changed = false;
        if (gamepad1.dpadUpWasPressed())    { invX  = !invX;  changed = true; }
        if (gamepad1.dpadRightWasPressed()) { invY  = !invY;  changed = true; }
        if (gamepad1.dpadDownWasPressed())  { invX2 = !invX2; changed = true; }
        if (changed) {
            OctoConfig.setEncoderDirection(q, OctoConfig.CH_X,  invX);
            OctoConfig.setEncoderDirection(q, OctoConfig.CH_Y,  invY);
            OctoConfig.setEncoderDirection(q, OctoConfig.CH_X2, invX2);
            if (!OctoConfig.calibrateImu(q, this)) {
                shout("localizer reset FAILED after a direction change. The pose is still "
                        + "using the OLD directions -- do not trust it.");
            }
            zero();
        }
    }

    private void zero() {
        q.setLocalizerPose(0, 0, 0f);
        q.resetAllPositions();
        rawX = rawY = rawX2 = 0;
        bx = by = headingDeg = 0;
    }

    // ------------------------------------------------------------------------ rendering

    private void render() {
        switch (check) {
            case FIRMWARE:      renderFirmware();  break;
            case PUSH_FORWARD:  renderForward();   break;
            case PUSH_SIDEWAYS: renderSideways();  break;
            case LTEST:         renderLTest();     break;
            case DONE:          renderDone();      break;
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

        telemetry.addLine("0/3 BOARD IDENTITY");
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
                enterCheck(Check.PUSH_FORWARD);
            } else {
                results.add("0 board   IMU CALIBRATION FAILED -- nothing below is meaningful");
                enterCheck(Check.DONE);
            }
        }
    }

    /**
     * The check that was missing from the repo, and the direct fix for "forward reads
     * negative X".
     *
     * Both parallel pods point the same way, so both must count up when the robot moves
     * forward. Pose x must go positive too: both candidate board frames put +X out the front,
     * so unlike Y this is safe to assert outright.
     */
    private void renderForward() {
        boolean moved = Math.abs(rawX) >= MIN_PUSH_COUNTS;
        String reason = null;
        if (!moved) {
            reason = String.format(Locale.US,
                    "ch0 moved only %d counts. Push at least ~100mm.", Math.abs(rawX));
        } else if (rawX < 0) {
            reason = "ch0 counts DOWN moving forward. Press DPAD UP to flip INVERT_X.";
        } else if (rawX2 < 0) {
            reason = "ch2 counts DOWN moving forward. Press DPAD DOWN to flip INVERT_X2.";
        } else if (Math.abs(rawX2) < MIN_PUSH_COUNTS / 2) {
            reason = "ch2 barely moved. Monitor pod unplugged, slipping, or on another port?";
        } else if (bx < 0) {
            reason = "pose x is NEGATIVE while moving forward, though ch0 counts up. "
                    + "The localizer is not reading port " + OctoConfig.CH_X + " as X.";
        }

        telemetry.addLine("1/3 PUSH THE ROBOT FORWARD");
        telemetry.addLine("B to zero, push forward ~1 to 2 ft in a straight line, A to accept");
        telemetry.addLine();
        telemetry.addData("ch0 X  raw", "%9d  must count UP     %s",
                rawX, verdict(rawX >= MIN_PUSH_COUNTS));
        telemetry.addData("ch2 X2 raw", "%9d  must count UP     %s",
                rawX2, verdict(rawX2 >= MIN_PUSH_COUNTS / 2));
        telemetry.addData("pose x",     "%9.1f mm  must be POSITIVE %s", bx, verdict(bx > 0));
        telemetry.addData("pose y",     "%9.1f mm  (ignore for now)", by);
        telemetry.addLine();
        renderToggleHelp();
        renderBlock(reason);

        boolean skip = gamepad1.xWasPressed();
        if (gamepad1.aWasPressed() || skip) {
            if (reason == null || skip) {
                results.add(reason == null
                        ? String.format(Locale.US,
                                "1 forward  ch0 +%d  ch2 +%d  x +%.0fmm  OK", rawX, rawX2, bx)
                        : "1 forward  SKIPPED");
                enterCheck(Check.PUSH_SIDEWAYS);
            } else {
                shout(reason);
            }
        }
    }

    /**
     * Liveness only. See the class comment for why this deliberately does not judge which
     * way ch1 should count: that answer depends on the board's handedness, and forcing it
     * here with INVERT_Y is the one mistake that corrupts the fusion instead of mirroring it.
     */
    private void renderSideways() {
        boolean moved = Math.abs(rawY) >= MIN_PUSH_COUNTS;
        String reason = moved ? null : String.format(Locale.US,
                "ch1 moved only %d counts. Push the robot sideways ~100mm or more.",
                Math.abs(rawY));

        telemetry.addLine("2/3 PUSH THE ROBOT SIDEWAYS  (either direction)");
        telemetry.addLine("B to zero, slide it sideways, A to accept");
        telemetry.addLine();
        telemetry.addData("ch1 Y raw", "%9d  just needs to MOVE  %s", rawY, verdict(moved));
        telemetry.addData("counting",  "%s when pushed that way",
                rawY >= 0 ? "UP" : "DOWN");
        telemetry.addLine();
        telemetry.addLine("This only proves the pod is alive and on port "
                + OctoConfig.CH_Y + ".");
        telemetry.addLine("Which way it SHOULD count depends on the board's handedness,");
        telemetry.addLine("which is undocumented. Check 3 settles that and INVERT_Y together.");
        telemetry.addLine();
        renderToggleHelp();
        renderBlock(reason);

        boolean skip = gamepad1.xWasPressed();
        if (gamepad1.aWasPressed() || skip) {
            if (reason == null || skip) {
                results.add(reason == null
                        ? String.format(Locale.US, "2 sideways ch1 %+d  alive  OK", rawY)
                        : "2 sideways SKIPPED");
                enterCheck(Check.LTEST);
            } else {
                shout(reason);
            }
        }
    }

    /**
     * The only check that combines rotation with translation, which is the one place a
     * handedness error can hide. Everything else either goes straight or spins in place, and
     * all of those pass a corrupted fusion happily.
     *
     * A self-consistent board reports y and heading with the SAME sign: +90 with +y (x fwd,
     * y left, CCW positive) or -90 with -y (x fwd, y right, CW positive). Both are valid
     * board frames; which one it is sets MIRROR_BOARD_FRAME.
     *
     * This is a SIGN test, so the turn does not need to be accurate. Anything recognisably a
     * quarter turn to the left works.
     */
    private void renderLTest() {
        boolean turned = Math.abs(headingDeg) > LTEST_MIN_HEADING_DEG;
        boolean travelled = Math.abs(by) > 100;
        boolean sameSign = Math.signum(by) == Math.signum(headingDeg);

        String reason = null;
        if (!turned) {
            reason = String.format(Locale.US,
                    "heading is only %.1f deg. Turn about a quarter turn LEFT.", headingDeg);
        } else if (!travelled) {
            reason = String.format(Locale.US,
                    "y is only %.0f mm. After turning, push it forward a foot or so.", by);
        } else if (!sameSign) {
            reason = "y and heading DISAGREE in sign. Two possible causes, and this test "
                    + "cannot tell them apart: (a) INVERT_Y is wrong, so press DPAD RIGHT "
                    + "to flip it, B, and redo; or (b) you turned RIGHT instead of LEFT.";
        }

        boolean boardMirror = headingDeg > 0;

        telemetry.addLine("3/3 L-TEST  (handedness + INVERT_Y)");
        telemetry.addLine("1. B to zero");
        telemetry.addLine("2. turn the robot about 90 deg LEFT, in place");
        telemetry.addLine("3. push it FORWARD about a foot");
        telemetry.addLine("4. A to accept.  Accuracy does not matter, only signs.");
        telemetry.addLine();
        telemetry.addData("heading", "%9.2f deg  %s", headingDeg, verdict(turned));
        telemetry.addData("y",       "%9.1f mm   %s", by, verdict(travelled));
        telemetry.addData("x",       "%9.1f mm   (near 0 if you did not drift)", bx);
        telemetry.addData("signs",   "%s", sameSign && turned && travelled
                ? "AGREE -- fusion is self-consistent" : "disagree or move too small");
        telemetry.addLine();
        if (reason == null) {
            telemetry.addData("board frame", "%s", boardMirror ? "y-LEFT / CCW+" : "y-RIGHT / CW+");
            telemetry.addData("=> MIRROR_BOARD_FRAME", "%b", boardMirror);
        }
        renderToggleHelp();
        renderBlock(reason);

        boolean skip = gamepad1.xWasPressed();
        if (gamepad1.aWasPressed() || skip) {
            if (reason == null || skip) {
                if (reason == null) {
                    mirror = boardMirror;
                    results.add("3 l-test   board " + (boardMirror ? "y-LEFT/CCW+" : "y-RIGHT/CW+")
                            + String.format(Locale.US, "  h %+.0f  y %+.0f  OK", headingDeg, by));
                } else {
                    results.add("3 l-test   SKIPPED -- kept mirror=" + mirror);
                }
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

    private void renderToggleHelp() {
        telemetry.addData("dpad UP / RIGHT / DOWN",
                "flip INVERT_X %b / INVERT_Y %b / INVERT_X2 %b", invX, invY, invX2);
        telemetry.addLine("  (a flip recalibrates the IMU -- hold the robot STILL for ~2s)");
        if (!dataOk) telemetry.addData("*** BAD READ", "crc/status invalid, %d so far", badReads);
    }

    private void renderBlock(String reason) {
        OctoConfig.renderAcceptBlock(telemetry, reason);
    }

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
