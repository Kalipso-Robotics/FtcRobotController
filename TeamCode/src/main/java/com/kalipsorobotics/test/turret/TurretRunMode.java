package com.kalipsorobotics.test.turret;

public enum TurretRunMode {
    RUN_WITH_POWER, // withinRange = true;
    RUN_USING_ODOMETRY_AND_LIMELIGHT,
    RUN_WHILE_SHOOTING,// withinRange = targetTicks - toleranceTicks;
    RUN_USING_LIMELIGHT,
    STOP, //withinRange = true;

}
