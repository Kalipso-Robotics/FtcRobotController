package com.kalipsorobotics.cameraVision;

public enum AllianceColor {

    RED(1),
    BLUE(-1);

    private final int polarity;

    AllianceColor(int polarity) {
        this.polarity = polarity;
    }

    public int getPolarity() {
        return polarity;
    }
}