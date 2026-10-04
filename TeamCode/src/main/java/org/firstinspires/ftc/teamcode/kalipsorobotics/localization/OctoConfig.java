package org.firstinspires.ftc.teamcode.kalipsorobotics.localization;

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
 * Pods: 3x goBILDA 4-Bar Odometry Pod, 32mm wheel, 2000 counts/rev.
 * Theoretical counts/mm = 2000 / (pi * 32) = 19.8944.
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
    public static final boolean INVERT_Y  = false;   // [2026-09-22] OctoStartup check 3
    public static final boolean INVERT_X2 = false;   // [2026-09-22] OctoStartup check 1

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
    public static final float POD_WHEEL_MM       = 32f;   // goBILDA 4-Bar [VERIFY WITH CALIPERS]

    public static final float CPM_THEORETICAL =
            (float) (POD_COUNTS_PER_REV / (Math.PI * POD_WHEEL_MM));

    // [2026-09-22] re-measured via OctoTune after the INVERT_X/INVERT_X2 fix (see the booleans
    // above): raw 24408 / 1828.8mm, 0.6% off theoretical. COUNTS_PER_MM_X2 now tracks it
    // closely (13.344 vs 13.346), confirming port 2 is alive again -- it read ~0 before the fix.
    public static final float COUNTS_PER_MM_X   = 19.911f;  // [2026-10-03] OctoTune pushes, see note above
    public static final float COUNTS_PER_MM_X2  = 19.911f;  // [2026-10-03] OctoTune pushes, see note above
    // [2026-09-22] Y re-measured after the L-test redo confirmed MIRROR_BOARD_FRAME/INVERT_Y
    // unchanged (still self-consistent, y-LEFT/CCW+): raw 16110 / 1219.2mm, 0.4% off theoretical.
    public static final float COUNTS_PER_MM_Y   = 19.911f;  // [2026-10-03] OctoTune pushes, see note above
    // [2026-09-26] HEADING re-measured with the net-signed OctoTune fix, 10 turns each way:
    // CW netDeg 3586.1 -> 1.0203, CCW netDeg -3587.7 -> 1.0199, averaged. (The 2026-09-22
    // value 1.0164 came from the old abs-sum measurement, which counted wobble as rotation.)
    public static final float IMU_HEADING_SCALAR = 1.0206f;      // [2026-10-03] 3 spins (CCW,CW,CCW) 1.02050/1.02073/1.02051, seed was 1.0201
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
     * after setting these, negate both. The SDK javadoc's own phrasing ("move the location of
     * the localizer's virtual TCP AWAY FROM the true location") reads as centre-minus-offset,
     * which would make these two the NEGATION of what is set below (63.5 -> -63.5, -152.4 ->
     * +152.4). CONFIRMED 2026-09-26 by replaying spin logs: offset = -(pod position).
     */
    // [2026-09-26] solved from the two OctoTune HEADING spins, not taped. Over exactly 10 turns
    // each pod reads (its distance from the spin centre) x (total radians):
    //   perpendicular pod: -95.57 / -95.18 mm forward  -> 95.4 mm BEHIND centre
    //   right pod (port 0): -146.64 / -151.13 mm left  -> 148.9 mm RIGHT of centre
    // Sign convention SETTLED by replaying the CW run's raw counts: the board's reported pose is
    // reproduced to 1 mm only if it places each pod at MINUS these offsets (the javadoc's
    // centre-minus-offset reading); the other sign misses by 400 mm. So offset = -(pod position).
    // The old 2026-09-22 tape values (63.5, -152.4) had X short and Y's sign backwards.
    // [2026-10-03] new drivetrain, 2 OctoTune HEADING spins (CW 11:31, CCW 11:32) re-run through
    // the fixed spinGeometry signs. Y pod 28.3 / 27.8 mm IN FRONT -> X = -28.0. Right pod radius
    // 72.8 / 83.5 is pivot-dependent (hand wander), so Y = track/2, assuming centred X pods.
    // [TAPE-VERIFY both] If a spin-in-place wanders MORE than with 0/0, negate both.
    public static final float TCP_OFFSET_MM_X   = -29.2f;  // [2026-10-03] mean of 5 spins (-28.3,-27.8,-31.3,-31.0,-27.7)
    public static final float TCP_OFFSET_MM_Y   = 77.3f;   // [2026-10-03] track/2, tape-verify

    /**
     * Distance between the two PARALLEL pods (port 0 and port 2), millimetres. Used only by the
     * heading monitor, which is never fused into the pose. The board never sees this value.
     * Solved by OctoTune's HEADING spin (see spinGeometry), not tape.
     */
    // [2026-10-03] 3 OctoTune HEADING spins: 154.66 (10/02 CCW), 154.27 (CW), 154.85 (CCW).
    public static final float TRACK_WIDTH_MM = 154.5f;  // [2026-10-03] mean of 6 spins (154.66,154.27,154.85,154.37,154.61,154.51)

    public static final int VELOCITY_INTERVAL_MS = 25;

    // ------------------------------------------------------------- test protocol
    //
    // Distances are declared in INCHES because that is what you lay out with a tape.
    // Everything internal stays in millimetres. Telemetry shows layout distances in inches
    // and error quantities in millimetres: you measure the first with a tape, you judge the
    // second as a small number.
    //
    // ONE 48 in lane along a field wall, used for both axes. The wall is the straightedge, so
    // twist error is ~0. X push: robot facing down the lane. Y push: set the robot down turned
    // 90 so its LEFT side faces down the lane and push it LEFT (ch1 counts up moving left,
    // the board's +y). Needs ~66 in of wall
    // (48 in travel + 18 in robot), under 3 tiles.
    //
    // PUSH_*_IN is the robot's TRAVEL, not the tape stop-to-stop: put a start stop behind the
    // robot, push it back against it, then measure from the robot's leading face at the start
    // to the far stop. Measure once, carefully.
    //
    // Error is ~2 mm fixed (tape read + seating) divided by travel: 0.16% at 48 in, 0.11% at
    // 72 in. Past 48 in the gain is below tile-compression noise, so 48 in is the knee.

    public static final double MM_PER_IN = 25.4;

    public static final double PUSH_X_IN = 48.0;   // robot travel, stage 2
    public static final double PUSH_Y_IN = 48.0;   // robot travel, stage 3 (same lane)

    public static final double PUSH_X_MM = PUSH_X_IN * MM_PER_IN;
    public static final double PUSH_Y_MM = PUSH_Y_IN * MM_PER_IN;

    /** Hand-turns for the HEADING stage: robot against a wall, spin, back against the wall. */
    public static final int    SPIN_ROTATIONS = 10;
    /** Deviation from an exact scalar of 1.0 that blocks accept in the HEADING stage. */
    public static final double TOL_SPIN_DEG    = 2.0;
    /** IMU vs pod-differential heading. Wider than TOL_SPIN_DEG: the monitor is itself noisy. */
    public static final double TOL_DISAGREE_DEG = 4.0;

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
     * EFFECTIVE RADIUS of each pod from N exact hand turns: r_eff = (mm the wheel rolled) /
     * (total radians turned), mm, about the spin centre. No sin/cos needed: a wheel rolling
     * along its own axis while the robot rotates by theta about a fixed point covers exactly
     * (perpendicular distance from that point to the wheel's line) x theta. theta comes from the
     * turn count, not the IMU, so IMU_HEADING_SCALAR does not enter.
     *
     * Returns {yPodBehind, xPodRight, x2PodLeft, trackWidth}:
     *   TCP_OFFSET_MM_X = yPodBehind, TCP_OFFSET_MM_Y = xPodRight, TRACK_WIDTH_MM = trackWidth.
     * Only the offset PERPENDICULAR to each pod's roll is observable; a pod cannot feel how far
     * along its own axis it sits, and nothing here uses those numbers.
     * Raw counts are in the BOARD's sign, so netDeg's sign picks the turn direction.
     *
     * Derived, not fitted. Board frame is x fwd / y LEFT / CCW+. A point at (px, py) moves at
     * w * (-py, px), so over theta: X pods roll -py*theta, the Y pod (counts up moving left)
     * rolls +px*theta. A CCW spin drives the right pod forward and the left pod back.
     * [2026-10-03] Signs were previously fitted to the 2026-09-26 logs, whose counts all had the
     * opposite sign to netDeg (pod/heading mismatch on the old build); that gave -81.5/-73.2/
     * -24.3/-154.66 on the 2026-10-02 spin instead of +81.5/+73.2/+24.3/+154.66.
     *
     * If the spin centre drifts in the body frame, the individual radii shift (one pod's radius
     * grows by what the other's shrinks), but xPodRight + x2PodLeft does NOT: track width is
     * robust to where you pivot, the two TCP offsets are not.
     */
    public static double[] spinGeometry(int rawX, int rawX2, int rawY, double netDeg, int turns) {
        double rad = Math.signum(netDeg) * turns * 2 * Math.PI;
        double right =  (rawX  / COUNTS_PER_MM_X)  / rad;
        double left  = -(rawX2 / COUNTS_PER_MM_X2) / rad;
        return new double[] { -(rawY / COUNTS_PER_MM_Y) / rad, right, left, right + left };
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
        // Recovers the I2C bus after a corrupted frame (e.g. ESD from a collision) instead of
        // silently wedging it. Recommended by the SDK's own SensorOctoQuadLocalization sample.
        q.setI2cRecoveryMode(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
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
        String error = calibrateImu(q, op::isStopRequested, (status, elapsedS) -> {
            op.telemetry.addLine("*** DO NOT TOUCH THE ROBOT -- calibrating IMU ***");
            op.telemetry.addData("elapsed", "%.1f s", elapsedS);
            op.telemetry.addData("status",  status);
            op.telemetry.addData("axis",    q.getLocalizerHeadingAxisChoice());
            op.telemetry.update();
        });
        if (error == null) {
            op.telemetry.addLine("IMU calibrated, localizer RUNNING.");
        } else {
            op.telemetry.addLine("*** " + error + " ***");
        }
        op.telemetry.update();
        return error == null;
    }

    /**
     * calibrateImu without an OpMode, for OctoQuadOdo. Returns null on success, else why it failed.
     * onPoll runs once per poll (status, seconds elapsed) for telemetry; may be null.
     */
    public static String calibrateImu(OctoQuad q, java.util.function.BooleanSupplier stopRequested,
                                      java.util.function.BiConsumer<OctoQuad.LocalizerStatus, Double> onPoll) {
        final double SETTLE_DWELL_S = 0.25;
        final int    CONSECUTIVE_RUNNING = 3;

        q.resetLocalizerAndCalibrateIMU();
        ElapsedTime t = new ElapsedTime();
        boolean sawBusy = false;
        int runningStreak = 0;

        while (!stopRequested.getAsBoolean()) {
            OctoQuad.LocalizerStatus status = q.getLocalizerStatus();

            if (status == OctoQuad.LocalizerStatus.FAULT_NO_IMU) {
                return "FAULT_NO_IMU -- the board cannot see its IMU. Check the board before running any Octo OpMode.";
            }

            if (status != OctoQuad.LocalizerStatus.RUNNING) {
                sawBusy = true;
                runningStreak = 0;
            } else if (sawBusy || t.seconds() > SETTLE_DWELL_S) {
                runningStreak++;
                if (runningStreak >= CONSECUTIVE_RUNNING) return null;
            }

            if (t.seconds() > IMU_CALIBRATE_TIMEOUT_S) {
                return "IMU calibration TIMED OUT (last status " + status
                        + "). Was the robot moved? Recalibration needs it dead still.";
            }

            if (onPoll != null) onPoll.accept(status, t.seconds());
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return "interrupted while calibrating";
            }
        }
        return "stop requested while calibrating";
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