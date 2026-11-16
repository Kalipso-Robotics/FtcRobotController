package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * The double-sided intake: side A (front, +X) and side B (back), plus the Flower on whichever
 * side ModuleConfig.FLOWER_SIDE says. Which side runs when is decided by the caller, not here.
 */
public class Intake {

    private final IntakeSide sideA;
    private final IntakeSide sideB;
    private final Flower flower;

    public Intake(OpModeUtilities opModeUtilities) {
        sideA = new IntakeSide(opModeUtilities, BeamBreakPosition.IntakeSide.A);
        sideB = new IntakeSide(opModeUtilities, BeamBreakPosition.IntakeSide.B);
        flower = new Flower(opModeUtilities, getSide(ModuleConfig.FLOWER_SIDE));
    }

    /** Direct-injection constructor (unit tests). The flower must already be attached to one of the sides. */
    Intake(IntakeSide sideA, IntakeSide sideB, Flower flower) {
        this.sideA = sideA;
        this.sideB = sideB;
        this.flower = flower;
    }

    public IntakeSide getSide(BeamBreakPosition.IntakeSide side) {
        return side == BeamBreakPosition.IntakeSide.A ? sideA : sideB;
    }

    /** The side facing a robot-frame forward (+X) direction or velocity: A for forward or zero, B for backward. */
    public IntakeSide getSideFacing(double robotForward) {
        return robotForward >= 0 ? sideA : sideB;
    }

    public Flower getFlower() {
        return flower;
    }

    /** Runs both sides, the closest equivalent of last year's single intake motor. */
    public void run(double power) {
        sideA.run(power);
        sideB.run(power);
    }

    /** Lowers both sides. Does not touch the flower. */
    public void lower() {
        sideA.lower();
        sideB.lower();
    }

    public void stop() {
        sideA.stop();
        sideB.stop();
    }

    /** Call once per loop. */
    public void update() {
        sideA.update();
        sideB.update();
    }

    /** Balls held, as last published by BeamBreakArray to SharedData. */
    public int getBallCount() {
        return SharedData.getBallCount();
    }

    /** True when the published beam-break count is at capacity. */
    public boolean isFull() {
        return SharedData.getBeamBreakData().isFull();
    }
}
