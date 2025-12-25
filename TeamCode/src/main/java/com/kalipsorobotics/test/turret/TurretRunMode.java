package com.kalipsorobotics.test.turret;

public enum TurretRunMode {
    RUN_WITH_POWER, // withinRange = true;
    RUN_USING_ODOMETRY, // withinRange = targetTicks - toleranceTicks;
    STOP, //withinRange = true;

}
