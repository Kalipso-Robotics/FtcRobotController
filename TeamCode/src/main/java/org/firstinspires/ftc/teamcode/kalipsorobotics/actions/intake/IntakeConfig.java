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
    public static double FLOWER_MOVE_MS = 400; // TODO-measure

    // Robot-frame forward speed beyond which the intake side switches; inside +/- this keeps the last side.
    public static double SIDE_SWITCH_VX_MM_PER_S = 100; // TODO-measure
}
