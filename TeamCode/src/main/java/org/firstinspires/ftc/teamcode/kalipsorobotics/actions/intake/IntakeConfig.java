package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import com.acmerobotics.dashboard.config.Config;

// Pattern: ShooterConfig
@Config
public class IntakeConfig {

    public static double shootTimeMS = 500;
    public static double intakeBallTimeMS = 5250;
    public static double intakePower = 1;
    public static double intakeFromRampTime = 2300;

    // Shoot feed sequence (PushBall): feed side A, wait, feed side B.
    public static double FEED_A_MS = 400; // TODO-measure
    public static double WAIT_BETWEEN_SIDES_MS = 150; // TODO-measure
    public static double FEED_B_MS = 400; // TODO-measure

    // Robot-frame forward speed beyond which the intake side switches; inside +/- this keeps the last side.
    public static double SIDE_SWITCH_VX_MM_PER_S = 100; // TODO-measure
}
