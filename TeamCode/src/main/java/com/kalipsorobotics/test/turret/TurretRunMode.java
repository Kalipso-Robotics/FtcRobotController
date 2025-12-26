package com.kalipsorobotics.test.turret;

public enum TurretRunMode {
    RUN_WITH_POWER, // withinRange = true;
    RUN_USING_ODOMETRY_AND_LIMELIGHT, // withinRange = targetTicks - toleranceTicks;
    RUN_USING_LIMELIGHT,
    STOP, //withinRange = true;

}
