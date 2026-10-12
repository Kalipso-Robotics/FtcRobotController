package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak;

/**
 * The four ball positions, declared in physical order along the path through the robot.
 *
 * The intake is double sided, so balls enter at either end and stack from the outside in:
 * A_OUTER fills first on the A side, B_OUTER fills first on the B side, and the two inner
 * positions fill last. The ordinal IS the physical index, and BeamBreakArray's plausibility
 * logic depends on that - do not reorder these without rereading it.
 */
public enum BeamBreakPosition {

    A_OUTER("beamBreakAOuter", IntakeSide.A, true),
    A_INNER("beamBreakAInner", IntakeSide.A, false),
    B_INNER("beamBreakBInner", IntakeSide.B, false),
    B_OUTER("beamBreakBOuter", IntakeSide.B, true);

    /** Which end of the robot this position is fed from. */
    public enum IntakeSide {
        A, B
    }

    public static final int COUNT = values().length;

    private final String deviceName;
    private final IntakeSide side;
    private final boolean outer;

    BeamBreakPosition(String deviceName, IntakeSide side, boolean outer) {
        this.deviceName = deviceName;
        this.side = side;
        this.outer = outer;
    }

    /** Name this sensor is configured under in the Robot Controller hardware map. */
    public String getDeviceName() {
        return deviceName;
    }

    public IntakeSide getSide() {
        return side;
    }

    /** True for the two positions at the mouths of the intake, which fill before the inner two. */
    public boolean isOuter() {
        return outer;
    }

    /** Physical index along the ball path, 0 at the A end through COUNT-1 at the B end. */
    public int index() {
        return ordinal();
    }

    public static BeamBreakPosition fromIndex(int index) {
        return values()[index];
    }

    /** Short label for telemetry, e.g. "A-out". */
    public String shortLabel() {
        return side.name() + (outer ? "-out" : "-in");
    }
}
