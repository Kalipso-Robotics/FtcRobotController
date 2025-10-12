package com.kalipsorobotics.modules.shooter;

public enum LaunchPosition {
    AUTO(1.0),
    NEAR(2.0),
    MIDDLE(3.0),
    WALL(4.0),
    BLUE(5.0),
    RED(6.0),
    MIDDLE_RED(7.0),
    MIDDLE_BLUE(8.0);

    private final double distanceToTargetMM;

    LaunchPosition(double value) {
        this.distanceToTargetMM = value;
    }

    public double getDistanceToTargetMM() {
        return distanceToTargetMM;
    }
}
