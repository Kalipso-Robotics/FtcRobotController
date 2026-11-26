package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeModuleConfig;

// Pattern: ShooterConfig
@Config
public class IntakeConfig {
    // Per-side settings. Side A is the front (+X, odometry side), side B the back.
    // Pivot: 0.44 is all the way up for A's servo, 0.55 for B's (init limits). Feed kF / kP: see FEED_RPS below.
    public static IntakeModuleConfig SIDE_A = new IntakeModuleConfig("intakeFront", "intakePivot1",
            DcMotorSimple.Direction.REVERSE, 0.68, 0.58, 0.44, 0.15598, 0.16623); // TODO-measure: kF, kP
    public static IntakeModuleConfig SIDE_B = new IntakeModuleConfig("intakeBack", "intakePivot2",
            DcMotorSimple.Direction.FORWARD, 0.30, 0.420, 0.55, 0.11183, 0.20235); // TODO-measure: kF, kP

    public static double intakeBallTimeMS = 5250; // TODO-measure
    public static double intakePower = 1; // TODO-measure: picking balls up
    public static double intakeFromRampTime = 2300; // TODO-measure

    // Shoot feed sequence (PushBall): side A and side B feed at the same time, each after its own delay.
    public static double SHOOT_FEED_POWER = 0.9; // TODO-measure: retune FEED_A_MS / FEED_B_MS whenever this changes
    public static double PRE_FEED_MS = 150; // TODO-measure: both sides spin this long against the closed stopper before it opens (they keep spinning while it opens)
    public static double STOPPER_OPEN_MS = 100; // TODO-measure: stopper's real open time; FEED_A_MS / FEED_B_MS start after it (KServo's own estimate is ~54ms, Axon speed)
    public static double FEED_A_MS = 1000; // TODO-measure
    public static double FEED_B_MS = 1000; // TODO-measure
    public static double FEED_A_DELAY_MS = 0; // TODO-measure
    public static double FEED_B_DELAY_MS = 0; // TODO-measure

    // Shot feed closed loop (PushBall feedA / feedB), in transfer motor RPS (ModuleConfig.INTAKE_TICKS_PER_REV): holds each
    // side's transfer wheels at FEED_RPS so balls loading the wheels or a sagging battery don't slow the feed.
    // Power = kF * target + kP * error (per side: SIDE_A / SIDE_B), times KMotor.voltageScale. FEED_RPS <= 0 = off (open loop at SHOOT_FEED_POWER).
    // The pre-feed stays open loop: it pushes a closed stopper, so a loop would see ~0 speed and go to full power.
    // Tune per side with IntakeTest. kF is measured free running (no balls): measured against a closed stopper it comes
    // out several times too high and the loop overdrives. Retune FEED_A_MS / FEED_B_MS after changing FEED_RPS.
    public static double FEED_RPS = 8; // TODO-measure: ~the old open-loop feed (0.9 power); motor free speed is ~27 RPS
    // Transfer "at speed" band for TransferReady (ShootAllAction): |RPS - FEED_RPS| <= this.
    // Hub velocity steps are ~0.2 RPS here (103.8 ticks/rev), so this can be tight.
    public static double FEED_RPS_TOLERANCE = 1.5; // TODO-measure
    // TransferReady gives up after this long (with balls against the closed stopper the transfer may never reach
    // FEED_RPS) and lets the shot go on, flagged timedOut(). Logged as Shooting_TransferReady event=timeout.
    public static double FEED_GATE_TIMEOUT_MS = 1500; // TODO-measure
    // ShootAllAction: how long the stopper stays open (IntakeRun feeding) before it closes and the intake stops.
    public static double shootTime = 1000; // TODO-measure: ms

    // Shooter RPS compensation (PushBall callers that pass a shooter error: ShootAllAction, so TeleOp too when on): during
    // the feed the transfer target becomes FEED_RPS + FEED_RPS_COMP_GAIN * (shooter target - actual RPS), so a sagging
    // flywheel gets the ball pushed in faster and a fast one slower -> same exit speed. Needs the closed loop
    // (FEED_RPS > 0). Shooter encoder steps are ~0.7 RPS, so expect ~0.7 * gain RPS of jitter.
    public static boolean FEED_RPS_COMP_ON = false;
    public static double FEED_RPS_COMP_GAIN = 0.3; // TODO-measure: transfer RPS per shooter RPS; raise until made shots stop drifting with the dip

    // How long a pivot (lower/raise) takes to finish moving.
    public static double PIVOT_MOVE_MS = 400; // TODO-measure

    // Flower arm: Axon Max in CR mode, no position feedback. It runs at the power for the time, then power is cut.
    // Retune the times whenever a power changes. Tune with FlowerCRTuner.
    public static double FLOWER_LOWER_MS = 1800; // TODO-measure: up -> down at FLOWER_DOWN_POWER
    public static double FLOWER_RAISE_MS = 1800; // TODO-measure: down -> up at FLOWER_UP_POWER
    public static double FLOWER_DOWN_POWER = 1; // TODO-measure
    public static double FLOWER_UP_POWER = -1; // TODO-measure: must be the opposite sign of DOWN (interlock uses the sign)
    public static boolean FLOWER_FLIP_DIRECTION = true; // read once at init

    // Robot-frame forward speed beyond which the intake side switches; inside +/- this keeps the last side.
    public static double SIDE_SWITCH_VX_MM_PER_S = 200; // TODO-measure: was 100 (~4 in/s), bumps into balls flipped it
    // vx must stay past SIDE_SWITCH_VX_MM_PER_S this long before the side switches (debounces bumps / bounces).
    public static double SIDE_SWITCH_CONFIRM_MS = 250; // TODO-measure: raise if bumps still flip it, lower if real reversals feel laggy

    // Full detection without beam breaks (IntakeFullDetector), same rule as DECODE's RunIntake: the running side is
    // full when its filtered current is >= STALL_CURRENT_MA AND its RPS is <= STALL_RPS, held for STALL_CONFIRM_MS.
    // Tune with IntakeMotorTest: run empty, then packed against the closed stopper, and read currentMA / RPS.
    public static double STALL_CURRENT_MA = 5517; // TODO-measure: DECODE's value
    // no balls -> 2780ma and 20.5RPS, 4 balls -> 6000ma and 11.9RPS
    public static double STALL_RPS = 14; // TODO-measure: DECODE used 50% of free speed (~27 RPS here)
    public static double STALL_CONFIRM_MS = 250; // TODO-measure: DECODE was 8 loops; one ball bumping through must not trip it
    public static double STALL_IGNORE_MS = 1000; // TODO-measure: spin-up after a side starts running (DECODE: 1 s)
    public static double STALL_FILTER = 0.3; // current smoothing per update, 0..1 (higher follows faster)
    // Ball estimate (IntakeFullDetector.estimateBalls): filtered current at or above BALL_n_MA reads as n balls.
    // Tune with IntakeMotorTest: hold RT, add one ball at a time, set each a bit below that step's filteredMA.
    // Only the estimate uses these; "full" is still the stall rule above.
    public static double BALL_1_MA = 2850; // TODO-measure
    public static double BALL_2_MA = 5000; // TODO-measure
    public static double BALL_3_MA = 5900; // TODO-measure
    public static double BALL_4_MA = 6100; // TODO-measure
}
