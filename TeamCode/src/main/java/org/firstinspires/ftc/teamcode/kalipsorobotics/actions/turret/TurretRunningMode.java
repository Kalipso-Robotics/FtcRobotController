package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret;

public enum TurretRunningMode {

    GO_TO_NEAREST_LIMIT(180),
    GO_TO_CENTER(0),
    GO_TO_OPPOSITE_LIMIT(-180);

    private final int degrees;

    TurretRunningMode(int degrees) {
        this.degrees = degrees;
    }

    public int getDegrees() {
        return degrees;
    }
}