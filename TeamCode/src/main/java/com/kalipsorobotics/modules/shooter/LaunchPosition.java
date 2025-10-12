package com.kalipsorobotics.modules.shooter;

public enum LaunchPosition {
    AUTO(1.0),
    NEAR(Math.hypot(Shooter.RED_TARGET_FROM_NEAR.getX(), Shooter.RED_TARGET_FROM_NEAR.getY())),
    MIDDLE(2286),
    WALL(1676.4),
    BLUE(3048),
    RED(150),
    MIDDLE_RED(1447.8),
    MIDDLE_BLUE(2413);

    private final double distanceToTargetMM;

    LaunchPosition(double value) {
        this.distanceToTargetMM = value;
    }

    public double getDistanceToTargetMM() {
        return distanceToTargetMM;
    }
}
