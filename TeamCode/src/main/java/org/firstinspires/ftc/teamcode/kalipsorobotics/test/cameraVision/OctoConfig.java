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
 * Pods: 3x goBILDA 4-Bar Odometry Pod, SKU 3110-0001-0002, 32mm wheel, 2000 counts/rev.
 * Theoretical counts/mm = 2000 / (pi * 32) = 19.89436789.
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
     * Set these from OctoQuadBringup, never by negating downstream. The localizer runs on the
     * board using the board's sign, so a fix in your pathing code leaves the fused pose wrong
     * while making raw counts look right. That passes a straight-line test and fails on curves.
     */
    public static final boolean INVERT_X  = false;  // [unverified 2026-09-19] set from Bringup
    public static final boolean INVERT_Y  = false;  // [unverified 2026-09-19] set from Bringup
    public static final boolean INVERT_X2 = false;  // [unverified 2026-09-19] set from Bringup

    /**
     * True when the board's frame is the mirror of yours and the pose must be flipped.
     * Determined by the L-test, Tune stage 5. Do not guess it.
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
    public static final boolean MIRROR_BOARD_FRAME = true;  // [unverified 2026-09-19] Tune stage 5

    public static final float CPM_THEORETICAL = 19.89436789f;

    public static final float COUNTS_PER_MM_X   = 19.89436789f;  // [SEED 2026-09-19] Tune stage 2
    public static final float COUNTS_PER_MM_X2  = 19.89436789f;  // [SEED 2026-09-19] Tune stage 2
    public static final float COUNTS_PER_MM_Y   = 19.89436789f;  // [SEED 2026-09-19] Tune stage 3
    public static final float TCP_OFFSET_MM_X   = 0.0f;          // [SEED 2026-09-19] Tune stage 4
    public static final float TCP_OFFSET_MM_Y   = 0.0f;          // [SEED 2026-09-19] Tune stage 4
    public static final float IMU_HEADING_SCALAR = 1.0f;         // [SEED 2026-09-19] Tune stage 5

    /**
     * Distance between the two PARALLEL pods (port 0 and port 2), millimetres. Used only by the
     * heading monitor. Measured in Tune stage 5 off the same 10-rotation spin as the scalar,
     * because a tape measure between two pod wheels is worse than the encoders themselves.
     */
    public static final float TRACK_WIDTH_MM = 300.0f;           // [SEED 2026-09-19] Tune stage 5

    public static final int VELOCITY_INTERVAL_MS = 25;

    // Test protocol and acceptance thresholds.
    public static final double PUSH_MM        = 2000.0;
    public static final int    SPIN_ROTATIONS = 10;
    public static final double SQUARE_MM      = 1000.0;
    public static final double STATIC_SECONDS = 60.0;

    public static final double TOL_LINEAR_PCT  = 1.0;
    public static final double TOL_SPIN_DEG    = 2.0;
    public static final double TOL_CLOSURE_MM  = 25.0;
    public static final double TOL_STATIC_MM   = 5.0;
    public static final double TOL_STATIC_DEG  = 0.5;
    /** IMU vs pod-differential heading. Wider than TOL_SPIN_DEG: the monitor is itself noisy. */
    public static final double TOL_DISAGREE_DEG = 4.0;

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
        out.x = b.posX_mm;                                    // [field names unverified]
        out.y = sign * b.posY_mm;
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
     * scale factors puts about 7 degrees of phantom rotation into a single 2 m straight push,
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

    // ----------------------------------------------------------------------- board setup

    /**
     * Push every constant above to the board. Runs at EVERY OpMode init: code is the source of
     * truth, flash is only a brownout backup. Flash alone loses the calibration silently on a
     * reflash or board swap with no record of what the numbers were.
     *
     * Order matters. Directions must be set before the localizer parameters mean anything.
     */
    public static void apply(OctoQuad q) {
        q.resetEverything();
        q.setChannelBankConfig(OctoQuad.ChannelBankConfig.ALL_QUADRATURE);  // [inferred, not verified]
        dir(q, CH_X,  INVERT_X);
        dir(q, CH_Y,  INVERT_Y);
        dir(q, CH_X2, INVERT_X2);
        q.setAllVelocitySampleIntervals(VELOCITY_INTERVAL_MS);
        // Only X and Y are given to the localizer. CH_X2 is read raw and never enters the pose.
        q.setAllLocalizerParameters(CH_X, CH_Y, COUNTS_PER_MM_X, COUNTS_PER_MM_Y,
                TCP_OFFSET_MM_X, TCP_OFFSET_MM_Y, IMU_HEADING_SCALAR, VELOCITY_INTERVAL_MS);
        q.saveParametersToFlash();
    }

    private static void dir(OctoQuad q, int ch, boolean invert) {
        q.setSingleEncoderDirection(ch, invert ? OctoQuad.EncoderDirection.REVERSE
                : OctoQuad.EncoderDirection.FORWARD);
    }

    /**
     * Zero the localizer and recalibrate the IMU. The robot must be COMPLETELY STILL: a bump
     * during calibration gives a bad heading zero with no error and no symptom until a spin
     * test fails.
     *
     * Gated on a button rather than a LocalizerStatus comparison because I could not verify
     * that enum's constant names. Once you confirm them, replace the button with a real status
     * check. A human deciding "that looks settled" is not a gate.
     */
    public static void calibrateImu(OctoQuad q, LinearOpMode op) {
        q.resetLocalizerAndCalibrateIMU();
        ElapsedTime t = new ElapsedTime();
        while (!op.isStopRequested()) {
            op.telemetry.addLine("*** DO NOT TOUCH THE ROBOT -- calibrating IMU ***");
            op.telemetry.addData("elapsed", "%.1f s", t.seconds());
            op.telemetry.addData("status",  q.getLocalizerStatus());
            op.telemetry.addData("axis",    q.getLocalizerHeadingAxisChoice());
            if (t.seconds() > 3.0) {
                op.telemetry.addLine("Press A when status reads settled.");
                if (op.gamepad1.aWasPressed()) { op.telemetry.update(); return; }
            }
            op.telemetry.update();
            op.sleep(50);
        }
    }
}