package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak;

/**
 * One beam break sensor's place in the robot: its hardware-map name, which intake side feeds it,
 * and its switching voltage. The list in BeamBreakConfig.SENSORS is ordered from the A end to the
 * B end, and BeamBreakArray's plausibility logic depends on that order - the list index IS the
 * physical index.
 *
 * The intake is double sided, so balls enter at either end and stack from the outside in.
 */
public final class BeamBreakPosition {

    /** Which end of the robot this position is fed from. */
    public enum IntakeSide {
        A, B
    }

    private final String deviceName;
    private final IntakeSide side;
    private final double thresholdV;

    public BeamBreakPosition(String deviceName, IntakeSide side, double thresholdV) {
        this.deviceName = deviceName;
        this.side = side;
        this.thresholdV = thresholdV;
    }

    /** Name this sensor is configured under in the Robot Controller hardware map. */
    public String getDeviceName() {
        return deviceName;
    }

    public IntakeSide getSide() {
        return side;
    }

    /** Switching voltage: roughly the midpoint of the blocked and clear voltages BeamBreakTest reports. */
    public double getThresholdV() {
        return thresholdV;
    }

    @Override
    public String toString() {
        return deviceName + "(" + side + ")";
    }
}
