package com.kalipsorobotics.test.turret;

public enum TurretRunMode {
    RUN_WITH_POWER, // withinRange = true;
    RUN_USING_ODOMETRY_AND_LL, // withinRange = targetTicks - toleranceTicks;
    RUN_USING_LL,
    STOP, //withinRange = true;

}
