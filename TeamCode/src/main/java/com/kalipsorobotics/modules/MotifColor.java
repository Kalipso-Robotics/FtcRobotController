package com.kalipsorobotics.modules;

public enum MotifColor {
    PURPLE, GREEN, NONE;


    public boolean isPurpleOrGreen() {
        return this == MotifColor.PURPLE || this == MotifColor.GREEN;
    }
}
