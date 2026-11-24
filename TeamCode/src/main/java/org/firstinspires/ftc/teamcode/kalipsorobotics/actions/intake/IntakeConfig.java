package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import com.acmerobotics.dashboard.config.Config;

// Pattern: ShooterConfig
@Config
public class IntakeConfig {

    public static double intakeBallTimeMS = 5250; // TODO-measure
    public static double intakePower = 1; // TODO-measure
    public static double intakeFromRampTime = 2300; // TODO-measure

    // Shoot feed sequence (PushBall): side A and side B feed at the same time, each after its own delay.
    public static double FEED_A_MS = 400; // TODO-measure
    public static double FEED_B_MS = 400; // TODO-measure
    public static double FEED_A_DELAY_MS = 0; // TODO-measure
    public static double FEED_B_DELAY_MS = 0; // TODO-measure

    // How long a pivot (lower/raise) and the Flower arm take to finish moving.
    public static double PIVOT_MOVE_MS = 400; // TODO-measure
    public static double FLOWER_MOVE_MS = 400; // TODO-measure: FlowerCRTuner, time for the arm to reach its stop

    // Flower arm: Axon Max in CR mode. Power is held after the move, pushing the arm into its hard stop,
    // so keep it as low as still reliably gets there and holds (high stall power heats the servo). Tune with FlowerCRTuner.
    public static double FLOWER_DOWN_POWER = 0; // TODO-measure
    public static double FLOWER_UP_POWER = 0; // TODO-measure: must be the opposite sign of DOWN (interlock uses the sign)
    public static boolean FLOWER_FLIP_DIRECTION = false; // read once at init

    // Robot-frame forward speed beyond which the intake side switches; inside +/- this keeps the last side.
    public static double SIDE_SWITCH_VX_MM_PER_S = 100; // TODO-measure
}
