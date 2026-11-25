package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import com.acmerobotics.dashboard.config.Config;

// Pattern: ShooterConfig
@Config
public class IntakeConfig {
    public static double intakeBallTimeMS = 5250; // TODO-measure
    public static double intakePower = 1; // TODO-measure: picking balls up
    public static double intakeFromRampTime = 2300; // TODO-measure

    // Shoot feed sequence (PushBall): side A and side B feed at the same time, each after its own delay.
    public static double SHOOT_FEED_POWER = 0.9; // TODO-measure: retune FEED_A_MS / FEED_B_MS whenever this changes
    public static double PRE_FEED_MS = 150; // TODO-measure: both sides spin this long against the closed stopper before it opens (they keep spinning while it opens)
    public static double FEED_A_MS = 900; // TODO-measure
    public static double FEED_B_MS = 900; // TODO-measure
    public static double FEED_A_DELAY_MS = 0; // TODO-measure
    public static double FEED_B_DELAY_MS = 0; // TODO-measure

    // Shot feed closed loop (PushBall feedA / feedB), in transfer motor RPS (ModuleConfig.INTAKE_TICKS_PER_REV): holds each
    // side's transfer wheels at FEED_RPS so balls loading the wheels or a sagging battery don't slow the feed.
    // Power = kF * target + kP * error, times KMotor.voltageScale. FEED_RPS <= 0 = off (open loop at SHOOT_FEED_POWER).
    // The pre-feed stays open loop: it pushes a closed stopper, so a loop would see ~0 speed and go to full power.
    // Tune per side with IntakeTest. kF is measured free running (no balls): measured against a closed stopper it comes
    // out several times too high and the loop overdrives. Retune FEED_A_MS / FEED_B_MS after changing FEED_RPS.
    public static double FEED_RPS = 8; // TODO-measure: ~the old open-loop feed (0.9 power); motor free speed is ~27 RPS
    public static double FEED_KF_A = 0.15598; // TODO-measure: power per RPS = 1 / free RPS at full power; read "kF est" in IntakeTest
    public static double FEED_KF_B = 0.11183; // TODO-measure
    public static double FEED_KP_A = 0.16623; // TODO-measure: power per RPS of error; raise until the loaded dip shrinks without hunting
    public static double FEED_KP_B = 0.20235; // TODO-measure
    // Transfer "at speed" band for TransferReady (ShootAllAction): |RPS - FEED_RPS| <= this.
    // Hub velocity steps are ~0.2 RPS here (103.8 ticks/rev), so this can be tight.
    public static double FEED_RPS_TOLERANCE = 1.5; // TODO-measure
    // TransferReady gives up after this long (with balls against the closed stopper the transfer may never reach
    // FEED_RPS) and lets the shot go on, flagged timedOut(). Logged as Shooting_TransferReady event=timeout.
    public static double FEED_GATE_TIMEOUT_MS = 1500; // TODO-measure

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
    public static double FLOWER_LOWER_MS = 400; // TODO-measure: up -> down at FLOWER_DOWN_POWER
    public static double FLOWER_RAISE_MS = 400; // TODO-measure: down -> up at FLOWER_UP_POWER
    public static double FLOWER_DOWN_POWER = 0; // TODO-measure
    public static double FLOWER_UP_POWER = 0; // TODO-measure: must be the opposite sign of DOWN (interlock uses the sign)
    public static boolean FLOWER_FLIP_DIRECTION = false; // read once at init

    // Robot-frame forward speed beyond which the intake side switches; inside +/- this keeps the last side.
    public static double SIDE_SWITCH_VX_MM_PER_S = 100; // TODO-measure
}
