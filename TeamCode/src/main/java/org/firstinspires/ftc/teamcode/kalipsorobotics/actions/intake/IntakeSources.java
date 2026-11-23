package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

public enum IntakeSources {
    FLOWER_1(1),
    FLOWER_2(1),
    GARDEN(1),
    TIP_ZONE_1(0),
    TIP_ZONE_2(0);

    private final double startValue;
    private double intakeValue;

    IntakeSources(double intakeValue) {
        this.startValue = intakeValue;
        this.intakeValue = intakeValue;
    }

    /** Puts every place back to its start value. Call at the start of auto: enum values stay in memory between runs. */
    public static void resetAll() {
        for (IntakeSources source : values()) {
            source.intakeValue = source.startValue;
        }
    }

    public double getIntakeValue() {
        return intakeValue;
    }

    public void subtractIntakeValue() {
        intakeValue--;
    }

    public void addIntakeValue() {
        intakeValue++;
    }


}

