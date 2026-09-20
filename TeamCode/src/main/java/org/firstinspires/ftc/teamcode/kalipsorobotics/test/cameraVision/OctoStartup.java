package org.firstinspires.ftc.teamcode.kalipsorobotics.test.cameraVision;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Run this FIRST, and run it again any time the wiring changes.
 *
 * Layout as built: port 0 right parallel (localizer X), port 1 middle perpendicular
 * (localizer Y), port 2 left parallel (heading monitor only).
 *
 * Two jobs, both of which you do while standing over the robot with a cable in your hand:
 *   INIT  -- is this an MK2 on firmware v3, talking over I2C?
 *   LOOP  -- are the signs right?
 *
 * On firmware: FTC SDK 11.0+ REQUIRES OctoQuad firmware v3.0+. The MK1 shipped with v2, the
 * MK2 with v3. An MK1 never upgraded will not work at all and will not tell you why. To
 * reflash: disconnect all power and data, hold BOOTSEL ('B' on the case), plug in micro-USB,
 * drag the image onto the drive that appears.
 * Images: github.com/DigitalChickenLabs/OctoQuad/tree/master/firmware
 *
 * WHAT TO CHECK HERE (and what NOT to):
 *
 *   X  : push FORWARD -> raw X and locX must INCREASE. Fix with INVERT_X.
 *   X2 : push FORWARD -> raw X2 must INCREASE too. Fix with INVERT_X2.
 *   turn RIGHT (clockwise) -> raw X2 (left pod) UP, raw X (right pod) DOWN.
 *
 *   Y  : just NOTE which way locY moves. Do NOT set INVERT_Y from a preference here.
 *
 * Why Y is different: the board's internal +Y direction is not documented, and its fusion
 * assumes a particular handedness between its X, its Y and the IMU heading. INVERT_Y has to
 * make the board self-consistent, not match your axis convention. Tune stage 5 (the L-test)
 * decides it: if that test reports the signs disagree, flip INVERT_Y and run it again.
 *
 * Flipping INVERT_Y to force +Y to point right would corrupt the fused pose on every curve.
 * Your convention (+X forward, +Y right, heading clockwise) is applied AFTER the board has
 * integrated, by OctoConfig.toRobotFrame().
 *
 * Fix a wrong sign with OctoConfig.INVERT_X / INVERT_Y. If HEADING is backwards there is no
 * flag for it: that means the IMU axis choice is not what we assumed, and the fix is different.
 *
 * LocalizerDataBlock field names below are the one thing I could not verify (the SDK javadoc
 * documents the class but not its members). If this does not compile, ctrl-click the class.
 */
@TeleOp(name = "OctoQuad 1 - Bringup", group = "OctoQuad")
public class OctoStartup extends LinearOpMode {

    @Override
    public void runOpMode() {

        OctoQuad q = hardwareMap.get(OctoQuad.class, OctoConfig.HARDWARE_NAME);

        telemetry.addData("chip id",   "0x%02X", q.getChipId());
        telemetry.addData("firmware",  q.getFirmwareVersionString());
        telemetry.addLine("  ^ MUST be 3.x. 2.x means an MK1 needing a reflash.");
        telemetry.addLine();
        telemetry.addData("localizer", localizerStatus(q));
        telemetry.addData("imu axis",  headingAxis(q));
        telemetry.addLine("  ^ an error on either means MK1: no IMU, no localizer,");
        telemetry.addLine("    and this whole package does not apply. Stop and re-plan.");
        telemetry.addLine();
        telemetry.addLine("Press START, then push the robot by hand.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        OctoConfig.apply(q);
        OctoConfig.calibrateImu(q, this);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);

        OctoQuad.LocalizerDataBlock loc = new OctoQuad.LocalizerDataBlock();
        OctoQuad.EncoderDataBlock   enc = new OctoQuad.EncoderDataBlock();
        OctoConfig.Pose robotPose = new OctoConfig.Pose();
        q.resetAllPositions();

        while (opModeIsActive()) {

            // ONE transaction per cycle. A full MK2 bulk read is ~3.8ms on a Control Hub;
            // reading pose and encoders separately doubles that for nothing.
            q.readLocalizerDataAndAllEncoderData(loc, enc);

            if (gamepad1.xWasPressed()) {
                q.resetAllPositions();
                q.setLocalizerPose(0, 0, 0f);
            }

            int rawX  = enc.positions[OctoConfig.CH_X];
            int rawY  = enc.positions[OctoConfig.CH_Y];
            int rawX2 = enc.positions[OctoConfig.CH_X2];
            double imuHeading = Math.toDegrees(loc.heading_rad);
            double podHeading = OctoConfig.monitorHeadingDeg(rawX, rawX2);

            telemetry.addLine("X = zero everything");
            telemetry.addLine();
            telemetry.addLine("FORWARD -> raw X and raw X2 both UP");
            telemetry.addLine("TURN RIGHT (CW) -> X2 UP, X DOWN");
            telemetry.addLine("Y: just note the direction. L-test sets INVERT_Y.");
            telemetry.addLine();
            telemetry.addData("raw X  (0 right)", "%8d cts", rawX);
            telemetry.addData("raw Y  (1 perp) ", "%8d cts", rawY);
            telemetry.addData("raw X2 (2 left) ", "%8d cts", rawX2);
            telemetry.addData("board", "x %8.1f  y %8.1f  h %7.2f  (board frame)",
                    loc.posX_mm, loc.posY_mm, imuHeading);
            OctoConfig.toRobotFrame(loc, robotPose);
            telemetry.addData("yours", "%s  (+X fwd, +Y right, CW+)", robotPose);
            telemetry.addData("crc  ", loc.crcOk ? "ok" : "*** FAILED ***");
            telemetry.addLine();

            // Independent heading from the two parallel pods, positive clockwise. Meaningless
            // until TRACK_WIDTH_MM and both parallel counts/mm are calibrated, so on a first
            // run ignore the magnitude and watch only that a RIGHT turn drives it positive.
            telemetry.addData("heading", "board %7.2f   pods(CW+) %7.2f",
                    imuHeading, podHeading);
            telemetry.addLine();

            // Cross-check: locX should track rawX / countsPerMM while driving straight.
            // Divergence means the localizer is reading a different port than you think.
            telemetry.addData("raw->mm", "X %8.1f (loc %8.1f)   Y %8.1f (loc %8.1f)",
                    rawX / OctoConfig.COUNTS_PER_MM_X, loc.posX_mm,
                    rawY / OctoConfig.COUNTS_PER_MM_Y, loc.posY_mm);
            telemetry.update();
        }
    }

    // MK2-only reads, wrapped so an MK1 reports a clear message instead of crashing.
    //
    // Deliberately NOT using getIMUDiagParam() or getUptimeMs(): those only exist in SDK
    // 12.0.0, not 11.x. This whole package targets 11.0 so it compiles on either. The
    // localizer API below is present from 11.0 on and is absent on an MK1, which is the
    // discrimination we actually need.
    private String localizerStatus(OctoQuad q) {
        try { return String.valueOf(q.getLocalizerStatus()); }
        catch (Exception e) { return "UNAVAILABLE (MK1?) " + e.getMessage(); }
    }

    private String headingAxis(OctoQuad q) {
        try { return String.valueOf(q.getLocalizerHeadingAxisChoice()); }
        catch (Exception e) { return "UNAVAILABLE (MK1?) " + e.getMessage(); }
    }
}