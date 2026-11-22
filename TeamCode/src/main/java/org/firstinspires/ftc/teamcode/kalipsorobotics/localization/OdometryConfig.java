package org.firstinspires.ftc.teamcode.kalipsorobotics.localization;

import com.acmerobotics.dashboard.config.Config;

@Config
public class OdometryConfig {

    public static double alphaLinear = 0.9; //0.3
    public static double alphaTheta = 0.7; //0.7

    /** true = OctoQuad pods (OctoQuadOdo), false = drive-port encoders (Odometry). Read by OpModeUtilities.startOdometry. */
    public static boolean USE_OCTOQUAD = true;
}
