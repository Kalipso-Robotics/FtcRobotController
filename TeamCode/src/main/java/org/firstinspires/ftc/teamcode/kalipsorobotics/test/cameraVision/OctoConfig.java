package org.firstinspires.ftc.teamcode.kalipsorobotics.test.cameraVision;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Constants for the OctoQuad FTC Ed. MK2 localizer, plus the act of pushing them to the board.
 * This is the only file your pathing code imports. The two OpModes are tools, not dependencies.
 *
 * RULE: every tuned number carries the DATE and the test that produced it. A constant with no
 * provenance is the one someone "fixes" at a competition with no record of what it used to be.
 *
 * Pods: 3x goBILDA Swingarm Odometry Pod, 48mm wheel, 2000 counts/rev.
 * Theoretical counts/mm = 2000 / (pi * 48) = 13.26291192.
 *
 * AXIS CONVENTION (yours): +X out the front of the robot, +Y out the RIGHT side,
 * heading positive CLOCKWISE seen from above. That is the aerospace/NED frame (x fwd,
 * y right, z down) and it is internally consistent. +Y right with CCW-positive heading
 * would NOT be: it is left-handed, and rotations stop composing.
 *
 * The board has its own frame. Convert at the boundary with toRobotFrame() below, never
 * by flipping an encoder. See the warning on MIRROR_BOARD_FRAME.
 *
 * LAYOUT (as built):
 *   port 0  right parallel   -> localizer X
 *   port 1  middle perpendicular -> localizer Y
 *   port 2  left parallel    -> heading MONITOR ONLY, never fused into the pose
 *
 * The MK2 localizer takes exactly one X port and one Y port and gets heading from its IMU, so
 * the third pod cannot improve the pose. It earns its channel as an independent heading check:
 * IMU heading drifts slowly and smoothly, pod-differential heading jumps when a pod skips, so
 * when the two disagree the SHAPE of the disagreement tells you which sensor is lying.
 */
public final class OctoConfig {

    private OctoConfig() {}

    public static final String HARDWARE_NAME = "octoquad";

    /** Right parallel pod -> localizer X. Pushing FORWARD must count UP. */
    public static final int CH_X  = 0;
    /** Middle perpendicular pod -> localizer Y. Pushing LEFT must count UP. */
    public static final int CH_Y  = 1;
    /** Left parallel pod -> monitor only. Pushing FORWARD must count UP. */
    public static final int CH_X2 = 2;

    /**
     * Set these from OctoStartup, never by negating downstream. The localizer runs on the
     * board using the board's sign, so a fix in your pathing code leaves the fused pose wrong
     * while making raw counts look right. That passes a straight-line test and fails on curves.
     *
     * OctoStartup owns every boolean in this file; OctoTune owns every float. Signs and
     * handedness are wiring facts and get settled in one session before a tape measure comes
     * out, because tuning against a wrongly-signed pod wastes the whole session.
     */
    // bring-up 2026-09-22 via OctoStartup (OctoStartup_LATEST.txt), redone after a 200%+
    // deviation in OctoTune showed X counting backwards -- ch0/ch2 direction changed from the
    // 2026-09-21 bring-up, presumably a re-seated or swapped connector.
    //   check 1 forward push: ch0 +8321, ch2 +8247, pose x +620mm
    //   check 2 sideways:     SKIPPED (still trusted from 2026-09-21, port 1 untouched)
    public static final boolean INVERT_X  = false;  // [2026-09-22] OctoStartup check 1
    public static final boolean INVERT_Y  = true;   // [2026-09-22] OctoStartup check 3
    public static final boolean INVERT_X2 = true;   // [2026-09-22] OctoStartup check 1

    /**
     * True when the board's frame is the mirror of yours and the pose must be flipped.
     * Determined by the L-test, OctoStartup check 3. Do not guess it.
     *
     * *** NEVER use INVERT_Y to express your axis convention. ***
     * The board integrates field pose as field += R(heading) * body_increment. Flipping one
     * encoder's sign without also flipping heading does not mirror that result, it corrupts
     * it: field_x becomes dx*cos + dy*sin instead of dx*cos - dy*sin, which is neither the
     * true value nor its negation. It looks perfect on every straight push and goes wrong the
     * instant heading leaves zero. INVERT_* is for a backwards-wired encoder, nothing else.
     *
     * A mirror flips Y and heading TOGETHER, which is what toRobotFrame() does, after the
     * board has finished integrating in the handedness its firmware was written for.
     */
    // [2026-09-22] OctoStartup check 3 redone: turned ~90 LEFT then pushed forward, board
    // reported heading +89 deg with y +430 mm. Same sign => board frame is y-LEFT /
    // CCW-positive, which is the mirror of ours, so the pose must be flipped. Confirms the
    // 2026-09-21 finding -- unchanged despite the X connector issue.
    public static final boolean MIRROR_BOARD_FRAME = true;

    /**
     * Pod geometry, as BUILT, not as ordered. CPM_THEORETICAL is derived from these two rather
     * than hardcoded, so OctoTune can invert the arithmetic and report the wheel diameter a
     * measurement IMPLIES. A 30% scale error is never rubber compression; it is almost always
     * these two numbers describing a pod you do not actually have. Check them with calipers
     * before believing anything downstream.
     *
     * goBILDA 4-Bar pod: 32mm wheel -> 19.894 counts/mm.
     * goBILDA Swingarm pod: 48mm wheel -> 13.263 counts/mm.
     */
    public static final float POD_COUNTS_PER_REV = 2000f;
    public static final float POD_WHEEL_MM       = 48f;   // goBILDA Swingarm [VERIFY WITH CALIPERS]

    public static final float CPM_THEORETICAL =
            (float) (POD_COUNTS_PER_REV / (Math.PI * POD_WHEEL_MM));

    // [2026-09-22] re-measured via OctoTune after the INVERT_X/INVERT_X2 fix (see the booleans
    // above): raw 24408 / 1828.8mm, 0.6% off theoretical. COUNTS_PER_MM_X2 now tracks it
    // closely (13.344 vs 13.346), confirming port 2 is alive again -- it read ~0 before the fix.
    public static final float COUNTS_PER_MM_X   = 13.34646f;     // [2026-09-22] OctoTune push X
    public static final float COUNTS_PER_MM_X2  = 13.34372f;     // [2026-09-22] OctoTune push X
    // [2026-09-22] Y re-measured after the L-test redo confirmed MIRROR_BOARD_FRAME/INVERT_Y
    // unchanged (still self-consistent, y-LEFT/CCW+): raw 16110 / 1219.2mm, 0.4% off theoretical.
    public static final float COUNTS_PER_MM_Y   = 13.21358f;     // [2026-09-22] OctoTune push Y
    // [2026-09-22] HEADING re-measured, slower this time: turnedDeg 3541.8 vs 3600 expected
    // (1.6% under, vs the prior run's 8.2% over), well clear of the wrap-safety gate.
    public static final float IMU_HEADING_SCALAR = 1.0164f;      // [2026-09-22] OctoTune HEADING
    /**
     * Tracking-centre offsets, BOARD frame (y-LEFT while MIRROR_BOARD_FRAME is true).
     *
     * Hand-measured, not solved. Draw a line through the middle of each tracking wheel parallel
     * to its direction of travel; where the two lines cross is the real TCP. Tape from there to
     * your robot centre. The SDK javadoc on setLocalizerTcpOffsetMM_X is explicit that this
     * "does not need to be extremely accurate since it will not affect the accuracy or
     * repeatability of the localizer algorithm", so a ruler beats a calibration routine.
     *
     * Only affects reported XY DURING A ROTATION. A straight push is offset-independent: every
     * point on a translating robot moves the same distance. Do not reach for these to explain a
     * straight-line scale error.
     *
     * The board's sign convention here is undocumented. If a spin-in-place makes XY wander more
     * after setting these, negate both.
     */
    // [2026-09-22] tape measure: 2.5in forward, 6in from centre to the port-0 (right) pod.
    // Forward is unaffected by MIRROR_BOARD_FRAME; left/right is negated because the board's
    // Y is LEFT-positive while the pod sits to the robot's RIGHT. Unverified by spin test --
    // per OctoConfig's own javadoc above, negate both if a spin-in-place in OctoTest makes
    // XY wander MORE than with these at zero.
    public static final float TCP_OFFSET_MM_X   = 63.5f;         // [2026-09-22] tape measure
    public static final float TCP_OFFSET_MM_Y   = -152.4f;       // [2026-09-22] tape measure

    /**
     * Distance between the two PARALLEL pods (port 0 and port 2), millimetres. Used only by the
     * heading monitor, which is never fused into the pose, so a tape measure between the two
     * pod wheels is good enough. The board never sees this value.
     */
    public static final float TRACK_WIDTH_MM = 298.45f;          // [2026-09-22] tape measure, 11.75in

    public static final int VELOCITY_INTERVAL_MS = 25;

    // ------------------------------------------------------------- test protocol
    //
    // Distances are declared in INCHES because that is what you lay out with a tape.
    // Everything internal stays in millimetres. Telemetry shows layout distances in inches
    // and error quantities in millimetres: you measure the first with a tape, you judge the
    // second as a small number.
    //
    // Sized for an L-SHAPED space of 4 x 3 tiles (24 in tiles):
    //   long leg  4 tiles = 96 in, minus an 18 in robot, minus slack -> 72 in of travel
    //   short leg 3 tiles = 72 in, minus an 18 in robot, minus slack -> 48 in of travel
    // The robot sits in the corner of the L facing down the long leg, with the short leg on
    // its RIGHT. Stage 2 pushes down the long leg, stage 3 down the short leg, no rotation
    // in between.
    //
    // Calibration accuracy is limited by (tape error / distance), so measure stop-to-stop
    // once, carefully. At 48 in a 1/16 in error is 0.13%; a 1/4 in error is 0.5%.

    public static final double MM_PER_IN = 25.4;

    public static final double PUSH_X_IN = 72.0;   // long leg, stage 2
    public static final double PUSH_Y_IN = 48.0;   // short leg, stage 3

    public static final double PUSH_X_MM = PUSH_X_IN * MM_PER_IN;
    public static final double PUSH_Y_MM = PUSH_Y_IN * MM_PER_IN;

    /** Hand-turns for the HEADING stage: robot against a wall, spin, back against the wall. */
    public static final int    SPIN_ROTATIONS = 10;
    /** Deviation from an exact scalar of 1.0 that blocks accept in the HEADING stage. */
    public static final double TOL_SPIN_DEG    = 2.0;
    /** IMU vs pod-differential heading. Wider than TOL_SPIN_DEG: the monitor is itself noisy. */
    public static final double TOL_DISAGREE_DEG = 4.0;

    // --------------------------------------------------------------- OctoTest tolerances
    //
    // Closure after a free drive. Deliberately loose: OctoTest is an open-ended drive and the
    // error it accumulates scales with how far and how much you turned, not with a fixed budget.

    public static final double TOL_TEST_CLOSURE_MM  = 50.0;
    public static final double TOL_TEST_HEADING_DEG = 3.0;

    /**
     * A sample period above this means the loop stalled (GC, I2C retry, telemetry flush).
     * The heading unwrapper assumes it sees every half-turn; a long gap breaks that
     * assumption, so stalls are flagged rather than silently trusted.
     */
    public static final double LOOP_STALL_MS = 100.0;

    /** Hard ceiling on IMU calibration. Past this something is wrong; say so, do not hang. */
    public static final double IMU_CALIBRATE_TIMEOUT_S = 15.0;

    // -------------------------------------------------------------------- pose boundary

    /** Pose in YOUR frame: +X forward, +Y right, heading positive clockwise, mm and degrees. */
    public static final class Pose {
        public double x, y, headingDeg;
        @Override public String toString() {
            return String.format(java.util.Locale.US,
                    "x %8.1f  y %8.1f  h %7.2f", x, y, headingDeg);
        }
    }

    /**
     * The one place the board's frame becomes yours. Call it once per read in your pathing
     * code and never touch the raw block anywhere else.
     *
     * Y and heading flip together or not at all. Flipping only one leaves a frame in which
     * rotating a pose and then translating it lands somewhere the math does not predict.
     */
    public static void toRobotFrame(OctoQuad.LocalizerDataBlock b, Pose out) {
        double sign = MIRROR_BOARD_FRAME ? -1.0 : 1.0;
        // Widened to double on the way in. The block's numeric field types are not documented
        // and at least one is narrower than float; keeping everything downstream in double
        // means no format string ever sees a boxed Short or Integer.
        out.x = (double) b.posX_mm;                           // [field names unverified]
        out.y = sign * (double) b.posY_mm;
        out.headingDeg = sign * Math.toDegrees(b.heading_rad);
    }

    // ------------------------------------------------------------------- heading monitor

    /**
     * Heading from the two parallel pods alone, degrees, CCW positive. Independent of the IMU.
     *
     * Sign follows YOUR convention: positive clockwise. Under a clockwise (rightward) turn the
     * left pod advances and the right pod retreats, so (left - right) / trackWidth is the angle.
     *
     * Each pod uses its OWN counts/mm. That is not pedantry: a 2% mismatch between the two
     * scale factors puts about 7 degrees of phantom rotation into a single 72 in straight push,
     * which would make this monitor fire constantly and teach you to ignore it.
     *
     * Never fuse this into the pose. Blending an estimate that drifts smoothly with one that
     * can step by degrees drags the good estimate toward the bad one. It is a check, not an input.
     */
    public static double monitorHeadingDeg(int rawX, int rawX2) {
        double mmRight = rawX  / COUNTS_PER_MM_X;
        double mmLeft  = rawX2 / COUNTS_PER_MM_X2;
        return Math.toDegrees((mmLeft - mmRight) / TRACK_WIDTH_MM);
    }

    /**
     * One loop's worth of heading change, wrapped to [-pi, pi]. Shared by OctoTune and OctoTest,
     * which both sum this to track total rotation: whether the board's heading_rad accumulates
     * or wraps at +/-pi is undocumented, and summing bounded deltas is correct either way,
     * provided no single loop iteration crosses whichever wrap point is real.
     */
    public static double wrapDeltaRad(double rawHeadingRad, double lastRawHeadingRad) {
        double d = rawHeadingRad - lastRawHeadingRad;
        while (d >  Math.PI) d -= 2 * Math.PI;
        while (d < -Math.PI) d += 2 * Math.PI;
        return d;
    }

    // ----------------------------------------------------------------------- board setup

    /**
     * Push every constant above to the board. Runs at EVERY OpMode init: code is the source of
     * truth, flash is only a brownout backup. Flash alone loses the calibration silently on a
     * reflash or board swap with no record of what the numbers were.
     *
     * Order matters. Directions must be set before the localizer parameters mean anything.
     *
     * NONE of this reaches the localizer until resetLocalizerAndCalibrateIMU() runs, which is
     * what calibrateImu() below does. That includes the encoder DIRECTIONS: flipping one
     * changes the reported counts at once but leaves the fused pose on the old sign until the
     * reset. Every caller of apply() must follow it with calibrateImu().
     *
     * Deliberately does NOT call resetEverything() or saveParametersToFlash().
     *
     * resetEverything() dumps the encoder counts and re-inits the localizer. Called from here
     * it would run at every OpMode init, including the TeleOp init straight after an auto,
     * silently zeroing any pose you meant to carry across that boundary. This method already
     * sets every parameter it cares about, so the reset buys nothing. OctoStartup calls it
     * once, where a clean slate is actually what you want.
     *
     * saveParametersToFlash() from here would burn a flash write on every single init for a
     * backup that only matters after a brownout. OctoTune saves once, at the end of tuning,
     * when the numbers have actually changed.
     */
    public static void apply(OctoQuad q) {
        q.setChannelBankConfig(OctoQuad.ChannelBankConfig.ALL_QUADRATURE);
        setEncoderDirection(q, CH_X,  INVERT_X);
        setEncoderDirection(q, CH_Y,  INVERT_Y);
        setEncoderDirection(q, CH_X2, INVERT_X2);
        q.setAllVelocitySampleIntervals(VELOCITY_INTERVAL_MS);
        // Only X and Y are given to the localizer. CH_X2 is read raw and never enters the pose.
        q.setAllLocalizerParameters(CH_X, CH_Y, COUNTS_PER_MM_X, COUNTS_PER_MM_Y,
                TCP_OFFSET_MM_X, TCP_OFFSET_MM_Y, IMU_HEADING_SCALAR, VELOCITY_INTERVAL_MS);
    }

    /** Shared with OctoStartup, which flips directions live on the board via dpad. */
    public static void setEncoderDirection(OctoQuad q, int ch, boolean invert) {
        q.setSingleEncoderDirection(ch, invert ? OctoQuad.EncoderDirection.REVERSE
                : OctoQuad.EncoderDirection.FORWARD);
    }

    /**
     * Zero the localizer and recalibrate the IMU. The robot must be COMPLETELY STILL: a bump
     * during calibration gives a bad heading zero with no error and no symptom until a spin
     * test fails.
     *
     * Returns true once the board reports RUNNING, false on timeout, missing IMU, or stop.
     *
     * This used to wait for a human to press A when the status "looked settled". A person
     * deciding that is not a gate, and a missed button press is exactly the failure this
     * whole rework exists to remove. LocalizerStatus.RUNNING is a real signal, so use it.
     *
     * Two things make the gate trustworthy rather than merely automatic:
     *
     * 1. The board can keep reporting the PREVIOUS status for a few milliseconds after
     *    resetLocalizerAndCalibrateIMU() lands. If the localizer was already RUNNING, polling
     *    immediately would see RUNNING and pass instantly without calibrating anything. So
     *    RUNNING is only believed after either observing a non-RUNNING state (proof the reset
     *    took effect) or waiting out a settling dwell.
     * 2. A corrupted I2C read could return RUNNING once by chance, so it must hold for
     *    several consecutive polls.
     */
    public static boolean calibrateImu(OctoQuad q, LinearOpMode op) {
        final double SETTLE_DWELL_S = 0.25;
        final int    CONSECUTIVE_RUNNING = 3;

        q.resetLocalizerAndCalibrateIMU();
        ElapsedTime t = new ElapsedTime();
        boolean sawBusy = false;
        int runningStreak = 0;

        while (!op.isStopRequested()) {
            OctoQuad.LocalizerStatus status = q.getLocalizerStatus();

            if (status == OctoQuad.LocalizerStatus.FAULT_NO_IMU) {
                op.telemetry.addLine("*** FAULT_NO_IMU -- the board cannot see its IMU. ***");
                op.telemetry.addLine("Nothing downstream of this is meaningful. Stop and");
                op.telemetry.addLine("check the board before running any other Octo OpMode.");
                op.telemetry.update();
                return false;
            }

            if (status != OctoQuad.LocalizerStatus.RUNNING) {
                sawBusy = true;
                runningStreak = 0;
            } else if (sawBusy || t.seconds() > SETTLE_DWELL_S) {
                runningStreak++;
                if (runningStreak >= CONSECUTIVE_RUNNING) {
                    op.telemetry.addLine("IMU calibrated, localizer RUNNING.");
                    op.telemetry.update();
                    return true;
                }
            }

            if (t.seconds() > IMU_CALIBRATE_TIMEOUT_S) {
                op.telemetry.addLine("*** IMU calibration TIMED OUT ***");
                op.telemetry.addData("last status", status);
                op.telemetry.addLine("Was the robot moved? Recalibration needs it dead still.");
                op.telemetry.update();
                return false;
            }

            op.telemetry.addLine("*** DO NOT TOUCH THE ROBOT -- calibrating IMU ***");
            op.telemetry.addData("elapsed", "%.1f s", t.seconds());
            op.telemetry.addData("status",  status);
            op.telemetry.addData("axis",    q.getLocalizerHeadingAxisChoice());
            op.telemetry.update();
            op.sleep(20);
        }
        return false;
    }

    // ------------------------------------------------------------- shared OpMode UI/autosave
    //
    // OctoStartup and OctoTune are both a loop of "accept or explain why not, autosave either
    // way" stages. This is the part of that loop that does not differ between them.

    /**
     * The whole "I pressed A and nothing happened" class of bug dies here, in one place. Shared
     * by OctoStartup and OctoTune, whose accept gates otherwise differ in every other way.
     */
    public static void renderAcceptBlock(org.firstinspires.ftc.robotcore.external.Telemetry telemetry,
                                          String reason) {
        if (reason == null) {
            telemetry.addLine(">>> A to accept.");
        } else {
            telemetry.addLine(">>> A is BLOCKED:");
            telemetry.addLine("    " + reason);
        }
    }

    /**
     * Overwrites a small, fixed-name file next to the timestamped CSV with just the results and
     * paste block, every time a stage/check is accepted or skipped. The timestamped CSV is a
     * full per-loop log meant for the run that produced it; this is the one file worth pulling
     * off the robot afterward, and it is always current without anyone copying numbers off a
     * screen. Shared by OctoStartup (OctoStartup_LATEST.txt) and OctoTune (OctoTune_LATEST.txt).
     */
    public static void writeLatestSnapshot(String csvPath, String fileName,
                                            org.firstinspires.ftc.robotcore.external.Telemetry telemetry,
                                            java.util.List<String> results, String[] pasteBlock) {
        java.io.File out = new java.io.File(new java.io.File(csvPath).getParentFile(), fileName);
        try (java.io.BufferedWriter w = new java.io.BufferedWriter(new java.io.FileWriter(out))) {
            w.write("# autosaved " + new java.text.SimpleDateFormat("yyyy-MM-dd HH:mm:ss", java.util.Locale.US)
                    .format(new java.util.Date()));
            w.newLine();
            for (String s : results) { w.write(s); w.newLine(); }
            w.newLine();
            for (String s : pasteBlock) { w.write(s); w.newLine(); }
        } catch (java.io.IOException e) {
            telemetry.addLine("autosave failed: " + e.getMessage());
        }
    }
}