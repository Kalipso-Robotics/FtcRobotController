package com.kalipsorobotics.cameraVision;

public enum AllianceSetup {

    RED(1),
    BLUE(-1);

    private final int polarity;

    AllianceSetup(int polarity) {
        this.polarity = polarity;
    }

    public int getPolarity() {
        return polarity;
    }
}