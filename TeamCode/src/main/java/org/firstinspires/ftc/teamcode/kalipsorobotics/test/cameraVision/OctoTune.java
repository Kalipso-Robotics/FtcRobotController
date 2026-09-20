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
 * Calibrates AND measures error in one pass, because they are the same measurement.
 *
 * A 2000mm push tells you two things from one number: the error against tolerance, and the
 * corrected counts/mm. There is no reason to push the robot twice.
 *
 * So you run this TWICE, total:
 *   Run 1 (seeded constants) -- errors are large, read off the proposed constants at the end.
 *   Paste them into OctoConfig, rebuild.
 *   Run 2 -- errors should now be inside tolerance. Run 2 IS the verification.
 *
 * Stage order is not arbitrary:
 *   1 STATIC   nothing else is trustworthy if the pose moves while the robot does not
 *   2 LINEAR X counts/mm before millimetres mean anything
 *   3 LINEAR Y
 *   4 TCP      before heading, or offset-induced translation inflates the heading error
 *   5 L-TEST   handedness, before any stage that trusts a rotated pose
 *   6 HEADING
 *   7 SQUARE   compounds all of the above
 *
 * You need: a tape-measured 2000mm run with hard stops at each end, and index marks on the
 * floor and chassis. The calibration cannot be more accurate than that measurement.
 *
 * Full time series goes to CSV, not just the summary, and that is the point: heading error
 * that accumulates linearly with angle is a scalar problem, error that jumps at one instant
 * is a pod skip or an I2C fault. A summary number cannot tell those apart.
 *   adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/RobotLogs ~/
 */
@TeleOp(name = "OctoQuad 2 - Tune + Verify", group = "OctoQuad")
public class OctoTune extends LinearOpMode {

    private enum Stage { STATIC, LINEAR_X, LINEAR_Y, TCP, LTEST, HEADING, SQUARE, DONE }

    private final OctoQuad.LocalizerDataBlock loc = new OctoQuad.LocalizerDataBlock();
    private final OctoQuad.EncoderDataBlock   enc = new OctoQuad.EncoderDataBlock();
    private final List<String> results = new ArrayList<>();
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime stageTimer = new ElapsedTime();

    private OctoQuad q;
    private KFileWriter csv;
    private Stage stage = Stage.STATIC;

    // Working values, seeded from OctoConfig and refined as stages are accepted.
    private float cpmX  = OctoConfig.COUNTS_PER_MM_X;
    private float cpmX2 = OctoConfig.COUNTS_PER_MM_X2;
    private float cpmY  = OctoConfig.COUNTS_PER_MM_Y;
    private float track = OctoConfig.TRACK_WIDTH_MM;
    private boolean mirror = OctoConfig.MIRROR_BOARD_FRAME;
    private float tcpX  = OctoConfig.TCP_OFFSET_MM_X;
    private float tcpY  = OctoConfig.TCP_OFFSET_MM_Y;
    private float scale = OctoConfig.IMU_HEADING_SCALAR;

    private double peak;          // worst excursion seen in the current stage
    private double headingDeg;    // IMU heading, cached per read
    private double podHeading;    // independent 2-parallel-pod heading, cached per read
    private double dist;          // cached per read

    @Override
    public void runOpMode() {

        q = hardwareMap.get(OctoQuad.class, OctoConfig.HARDWARE_NAME);
        OctoConfig.apply(q);

        // vvv GUESSED SIGNATURE: I do not have OpModeUtilities. One line to fix. vvv
        csv = new KFileWriter("OctoQuadTune", new OpModeUtilities(hardwareMap, this, telemetry));
        // ^^^
        csv.writeLine(String.format(Locale.US,
                "# seeds cpmX=%.5f cpmX2=%.5f cpmY=%.5f tcpX=%.2f tcpY=%.2f scale=%.6f track=%.1f",
                cpmX, cpmX2, cpmY, tcpX, tcpY, scale, track));
        csv.writeLine("stage,t_s,x_mm,y_mm,imu_deg,pod_deg,disagree_deg,"
                + "rawX,rawY,rawX2,crcOk,loop_ms");

        telemetry.addLine("Tune + Verify.");
        telemetry.addData("Need", "a taped %.0f mm run with hard stops, and floor index marks",
                OctoConfig.PUSH_MM);
        telemetry.addLine("Stage 1 needs the robot untouched. Press START.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) { csv.close(); return; }

        OctoConfig.calibrateImu(q, this);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);

        runtime.reset();
        enterStage(Stage.STATIC);

        ElapsedTime loopTimer = new ElapsedTime();
        try {
            while (opModeIsActive()) {
                double loopMs = loopTimer.milliseconds();
                loopTimer.reset();

                q.readLocalizerDataAndAllEncoderData(loc, enc);
                headingDeg = Math.toDegrees(loc.heading_rad);   // [field names unverified]
                podHeading = podHeading();
                dist = Math.hypot(loc.posX_mm, loc.posY_mm);
                peak = Math.max(peak, dist);

                if (stage != Stage.DONE) {
                    csv.writeLine(String.format(Locale.US,
                            "%s,%.3f,%.2f,%.2f,%.4f,%.4f,%.4f,%d,%d,%d,%b,%.2f",
                            stage, runtime.seconds(), loc.posX_mm, loc.posY_mm,
                            headingDeg, podHeading, headingDeg - podHeading,
                            enc.positions[OctoConfig.CH_X], enc.positions[OctoConfig.CH_Y],
                            enc.positions[OctoConfig.CH_X2], loc.crcOk, loopMs));
                }

                if (gamepad1.bWasPressed()) zero();
                render();
                telemetry.update();
            }
        } finally {
            csv.close();
        }
    }

    // ------------------------------------------------------------------ stage machine

    private void render() {
        switch (stage) {
            case STATIC:   renderStatic();  break;
            case LINEAR_X: renderLinear(true);  break;
            case LINEAR_Y: renderLinear(false); break;
            case TCP:      renderTcp();     break;
            case LTEST:    renderLTest();  break;
            case HEADING:  renderHeading(); break;
            case SQUARE:   renderSquare();  break;
            case DONE:     renderDone();    break;
        }
    }

    private void renderStatic() {
        double remaining = OctoConfig.STATIC_SECONDS - stageTimer.seconds();
        telemetry.addLine("1/7 STATIC DRIFT -- DO NOT TOUCH THE ROBOT");
        telemetry.addData("remaining", "%.0f s", remaining);
        telemetry.addData("drift",   "%6.2f mm  (limit %.1f)", peak, OctoConfig.TOL_STATIC_MM);
        telemetry.addData("heading", "%6.3f deg (limit %.1f)",
                Math.abs(headingDeg), OctoConfig.TOL_STATIC_DEG);
        if (remaining <= 0) {
            record("1 static drift",   peak, OctoConfig.TOL_STATIC_MM,  "mm");
            record("1 static heading", Math.abs(headingDeg), OctoConfig.TOL_STATIC_DEG, "deg");
            if (peak > OctoConfig.TOL_STATIC_MM) {
                results.add("  ^ pose moves while robot does not. Mounting flex or vibration.");
                results.add("    Every later stage inherits this. Fix it before tuning around it.");
            }
            enterStage(Stage.LINEAR_X);
        }
    }

    /**
     * reported = counts / countsPerMM, so to make reported equal actual:
     *   cpm_new = cpm_old * (reported / actual)
     * X and Y are corrected independently and will not match. The two pods sit under
     * different loads and their rubber compresses differently.
     */
    private void renderLinear(boolean isX) {
        double reported = isX ? loc.posX_mm : loc.posY_mm;
        double errPct = (reported - OctoConfig.PUSH_MM) / OctoConfig.PUSH_MM * 100.0;
        float current = isX ? cpmX : cpmY;
        float proposed = (float) (current * (reported / OctoConfig.PUSH_MM));
        double deviation = Math.abs(proposed - OctoConfig.CPM_THEORETICAL)
                / OctoConfig.CPM_THEORETICAL * 100.0;

        telemetry.addLine((isX ? "2/7 LINEAR X (push FORWARD)" : "3/7 LINEAR Y (push RIGHT)"));
        telemetry.addLine("B at the start stop, push to the end stop, A to record");
        if (!isX) telemetry.addLine("Push RIGHT: +Y is out the right side in your convention.");
        telemetry.addData("reported", "%8.1f mm  (actual %.0f)", reported, OctoConfig.PUSH_MM);
        telemetry.addData("error",    "%8.2f %%   (limit %.1f)", errPct, OctoConfig.TOL_LINEAR_PCT);
        telemetry.addData("proposed counts/mm", "%.5f", proposed);
        telemetry.addData("off theoretical",    "%.2f %%", deviation);
        telemetry.addData("cross-axis", "%8.1f mm  should be ~0", isX ? loc.posY_mm : loc.posX_mm);

        // The monitor pod travelled the same 2000mm, so this push calibrates it for free.
        // It needs its OWN scale factor: a 2% mismatch against the X pod injects about 7 deg
        // of phantom rotation into one straight push, which would make the monitor useless.
        float proposedX2 = 0f;
        if (isX) {
            proposedX2 = (float) (enc.positions[OctoConfig.CH_X2] / OctoConfig.PUSH_MM);
            telemetry.addData("proposed counts/mm X2", "%.5f  (monitor pod, port 2)", proposedX2);
        }
        if (deviation > 5.0) {
            telemetry.addLine("*** >5% off theoretical. NOT rubber compression. ***");
            telemetry.addLine("Suspect a slipping pod, wrong wheel, or wrong channel.");
        }

        if (gamepad1.aWasPressed()) {
            record(isX ? "2 linear X" : "3 linear Y", Math.abs(errPct),
                    OctoConfig.TOL_LINEAR_PCT, "%");
            if (isX) {
                cpmX = proposed; q.setLocalizerCountsPerMM_X(cpmX);
                cpmX2 = proposedX2;   // monitor only, never sent to the board
            } else {
                cpmY = proposed; q.setLocalizerCountsPerMM_Y(cpmY);
            }
            enterStage(isX ? Stage.LINEAR_Y : Stage.TCP);
        }
    }

    /**
     * Tracking-centre offsets by live adjustment rather than from CAD.
     *
     * A pure rotation about a correctly placed tracking centre reports no translation, so
     * whatever translation you see is rotation coupled in by the offset. Nudge until the peak
     * collapses. This also resolves the sign convention by hand, which matters because the
     * convention is not documented in anything I could retrieve: if a direction makes it
     * worse, go the other way.
     */
    private void renderTcp() {
        telemetry.addLine("4/7 TRACKING-CENTRE OFFSETS");
        telemetry.addLine("Spin the robot in place, several turns, BOTH directions.");
        telemetry.addData("peak excursion", "%8.1f mm  <-- MINIMISE (target <10)", peak);
        telemetry.addData("tcpX", "%8.2f mm  dpad up/down", tcpX);
        telemetry.addData("tcpY", "%8.2f mm  dpad left/right", tcpY);
        telemetry.addLine("hold LEFT BUMPER for 0.1mm steps | B zero | A accept");
        telemetry.addLine("If a direction makes it worse, go the other way.");

        float step = gamepad1.left_bumper ? 0.1f : 1.0f;
        boolean changed = false;
        if (gamepad1.dpadUpWasPressed())    { tcpX += step; changed = true; }
        if (gamepad1.dpadDownWasPressed())  { tcpX -= step; changed = true; }
        if (gamepad1.dpadRightWasPressed()) { tcpY += step; changed = true; }
        if (gamepad1.dpadLeftWasPressed())  { tcpY -= step; changed = true; }

        if (changed) {
            q.setLocalizerTcpOffsetMM_X(tcpX);
            q.setLocalizerTcpOffsetMM_Y(tcpY);
            zero();
        }
        if (gamepad1.aWasPressed()) {
            results.add(String.format(Locale.US, "4 tcp              peak %6.1f mm", peak));
            enterStage(Stage.LTEST);
        }
    }

    /**
     * The L-test: the only stage that combines rotation and translation, which is the one
     * place a handedness error can hide. Every other stage either goes straight or spins in
     * place, and all of them pass a corrupted fusion.
     *
     * Zero, turn the robot 90 degrees to its LEFT, push forward 1 m, read the pose.
     *
     * A self-consistent board reports heading and Y with the SAME sign: either +90 and +1000
     * (x fwd, y left, CCW positive) or -90 and -1000 (x fwd, y right, CW positive). Both are
     * valid frames, and which one it is sets MIRROR_BOARD_FRAME.
     *
     * Opposite signs mean the fusion itself is broken, almost always because an encoder was
     * inverted to force a coordinate convention. No constant fixes that.
     */
    private void renderLTest() {
        double x = loc.posX_mm, y = loc.posY_mm;
        boolean consistent = Math.signum(y) == Math.signum(headingDeg)
                && Math.abs(headingDeg) > 45 && Math.abs(y) > 300;
        boolean mirror = headingDeg > 0;

        telemetry.addLine("5/7 L-TEST (handedness)");
        telemetry.addLine("B to zero, turn the robot 90 deg LEFT in place,");
        telemetry.addLine("then push it FORWARD 1000 mm, then A.");
        telemetry.addLine();
        telemetry.addData("heading", "%8.2f deg   expect about +/-90", headingDeg);
        telemetry.addData("y",       "%8.1f mm    expect about +/-1000", y);
        telemetry.addData("x",       "%8.1f mm    expect near 0", x);
        telemetry.addLine();
        if (consistent) {
            telemetry.addLine("CONSISTENT. Board frame is "
                    + (mirror ? "y-LEFT / CCW+" : "y-RIGHT / CW+"));
            telemetry.addData("=> MIRROR_BOARD_FRAME", "%b", mirror);
        } else {
            telemetry.addLine("*** heading and y disagree in sign, or the move was too small.");
            telemetry.addLine("If the move was full size, the fusion is BROKEN: an encoder has");
            telemetry.addLine("been inverted to force a convention. Clear INVERT_Y and redo.");
        }
        telemetry.addLine("Do NOT record until this reads CONSISTENT.");

        if (gamepad1.aWasPressed() && consistent) {
            this.mirror = mirror;
            results.add("5 l-test            board " + (mirror ? "y-LEFT/CCW+" : "y-RIGHT/CW+")
                    + ", |x| " + String.format(Locale.US, "%.0f", Math.abs(x)) + " mm");
            enterStage(Stage.HEADING);
        }
    }

    /**
     * reported = raw * scale, so: scale_new = scale_old * (actual / reported)
     *
     * Ten rotations, not one: a 0.2 deg/rotation error is invisible in a single turn and
     * ruins an auto over a match. With two pods there is no second sensor to check against,
     * so the reference here is the floor, not another number.
     */
    private void renderHeading() {
        // Direction-agnostic. The board's heading sign is whatever the L-test found, so
        // requiring a particular spin direction here would just be one more thing to get
        // backwards. Spin either way, consistently, and the magnitudes do the work.
        double actual = 360.0 * OctoConfig.SPIN_ROTATIONS;
        double swept = Math.abs(headingDeg);
        float proposed = swept == 0 ? scale : (float) (scale * (actual / swept));

        telemetry.addLine("6/7 HEADING SCALAR");
        telemetry.addData("do", "align to mark, B, spin %d full turns EITHER way back to it, A",
                OctoConfig.SPIN_ROTATIONS);
        telemetry.addData("reported", "%8.2f deg  (|swept| vs actual %.0f)", headingDeg, actual);
        telemetry.addData("error",    "%8.2f deg  (limit %.1f)",
                swept - actual, OctoConfig.TOL_SPIN_DEG);
        telemetry.addData("proposed scalar", "%.6f", proposed);

        // Track width from the same spin. A tape measure between two pod wheels is worse than
        // the encoders themselves, so derive it rather than measure it.
        //   (mmRight - mmLeft) = trackWidth * sweptAngleRadians
        double sweptMm = Math.abs(enc.positions[OctoConfig.CH_X2] / cpmX2
                - enc.positions[OctoConfig.CH_X]  / cpmX);
        float proposedTrack = (float) (sweptMm / Math.toRadians(actual));
        telemetry.addData("proposed track width", "%.1f mm  (monitor only)", proposedTrack);
        telemetry.addLine();
        telemetry.addLine("Heading must move MONOTONICALLY through all 10 turns. If it stalls");
        telemetry.addLine("or reverses mid-spin, that is the IMU, not the scalar.");

        if (gamepad1.aWasPressed()) {
            record("6 spin heading", Math.abs(swept - actual),
                    OctoConfig.TOL_SPIN_DEG, "deg");
            scale = proposed;
            track = proposedTrack;
            q.setLocalizerImuHeadingScalar(scale);
            enterStage(Stage.SQUARE);
        }
    }

    private void renderSquare() {
        telemetry.addLine("7/7 SQUARE LOOP CLOSURE");
        telemetry.addData("do", "B, push around a %.0f mm square, return to the SAME spot and"
                + " orientation, A", OctoConfig.SQUARE_MM);
        telemetry.addData("closure", "%8.1f mm  (limit %.1f)", dist, OctoConfig.TOL_CLOSURE_MM);
        telemetry.addData("x / y",   "%8.1f / %8.1f mm", loc.posX_mm, loc.posY_mm);
        telemetry.addData("heading", "%8.2f deg  (pods say %7.2f)", headingDeg, podHeading);
        telemetry.addData("disagree", "%8.2f deg  (limit %.1f)",
                headingDeg - podHeading, OctoConfig.TOL_DISAGREE_DEG);
        telemetry.addLine("The other stages each isolate one axis. This one compounds them.");
        telemetry.addLine("Disagreement that GREW SMOOTHLY = scalar or track width.");
        telemetry.addLine("Disagreement that JUMPED = a pod skipped. Find it in the CSV.");

        if (gamepad1.aWasPressed()) {
            record("7 square closure", dist, OctoConfig.TOL_CLOSURE_MM, "mm");
            record("7 square heading", Math.abs(headingDeg), OctoConfig.TOL_SPIN_DEG, "deg");
            record("7 imu vs pods", Math.abs(headingDeg - podHeading),
                    OctoConfig.TOL_DISAGREE_DEG, "deg");
            q.saveParametersToFlash();
            for (String s : pasteBlock()) csv.writeLine(s);
            enterStage(Stage.DONE);
        }
    }

    private void renderDone() {
        telemetry.addLine("=== RESULTS ===");
        for (String r : results) telemetry.addLine(r);
        telemetry.addLine();
        for (String s : pasteBlock()) telemetry.addLine(s);
        telemetry.addLine();
        telemetry.addLine("Paste into OctoConfig, rebuild, run this again.");
        telemetry.addLine("Second run is the verification. CSV: " + csv.getPath());
    }

    // ------------------------------------------------------------------------ helpers

    private void enterStage(Stage s) {
        stage = s;
        stageTimer.reset();
        zero();
        csv.writeLine("# --- stage " + s);
    }

    /**
     * Heading from the two parallel pods using the WORKING constants, so it stays meaningful
     * mid-tune while OctoConfig still holds the seeds. Monitor only: never fused into the pose,
     * because blending a smoothly-drifting estimate with one that can step by degrees drags the
     * good estimate toward the bad one.
     */
    private double podHeading() {
        double mmRight = enc.positions[OctoConfig.CH_X]  / cpmX;
        double mmLeft  = enc.positions[OctoConfig.CH_X2] / cpmX2;
        // Matches OctoConfig.monitorHeadingDeg: positive CLOCKWISE, your convention.
        return Math.toDegrees((mmLeft - mmRight) / track);
    }

    private void zero() {
        q.setLocalizerPose(0, 0, 0f);
        q.resetAllPositions();
        peak = 0;
    }

    private void record(String name, double value, double limit, String unit) {
        String line = String.format(Locale.US, "%-18s %8.2f %-3s limit %6.2f  %s",
                name, value, unit, limit, value <= limit ? "PASS" : "FAIL");
        results.add(line);
        csv.writeLine("# RESULT " + line);
    }

    private String[] pasteBlock() {
        String today = new java.text.SimpleDateFormat("yyyy-MM-dd", Locale.US)
                .format(new java.util.Date());
        return new String[] {
                "// calibrated " + today + " via OctoQuadTune",
                String.format(Locale.US, "COUNTS_PER_MM_X    = %.5ff;", cpmX),
                String.format(Locale.US, "COUNTS_PER_MM_X2   = %.5ff;", cpmX2),
                String.format(Locale.US, "COUNTS_PER_MM_Y    = %.5ff;", cpmY),
                String.format(Locale.US, "TCP_OFFSET_MM_X    = %.2ff;", tcpX),
                String.format(Locale.US, "TCP_OFFSET_MM_Y    = %.2ff;", tcpY),
                String.format(Locale.US, "IMU_HEADING_SCALAR = %.6ff;", scale),
                String.format(Locale.US, "TRACK_WIDTH_MM     = %.1ff;", track),
                String.format(Locale.US, "MIRROR_BOARD_FRAME = %b;", mirror),
        };
    }
}