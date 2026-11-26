package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.IShooterPredictor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.ShooterInterpolationDataLookup;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * The double-sided intake: module A (front, +X) and module B (back), plus the Flower on whichever
 * side ModuleConfig.FLOWER_SIDE says. Both-side commands are here; getModule() hands out one side
 * for callers that run them separately. Which side runs when is decided by the caller, not here.
 */
public class Intake {

    private final IntakeModule sideA;
    private final IntakeModule sideB;
    private final FlowerMech flowerMech;


    public Intake(OpModeUtilities opModeUtilities) {
        sideA = new IntakeModule(opModeUtilities, BeamBreakPosition.IntakeSide.A, IntakeConfig.SIDE_A);
        sideB = new IntakeModule(opModeUtilities, BeamBreakPosition.IntakeSide.B, IntakeConfig.SIDE_B);
        flowerMech = new FlowerMech(opModeUtilities, getModule(ModuleConfig.FLOWER_SIDE));
    }

    /** Direct-injection constructor (unit tests, public so actions tests can build one). The flower must already be attached to one of the modules. */
    public Intake(IntakeModule sideA, IntakeModule sideB, FlowerMech flowerMech) {
        this.sideA = sideA;
        this.sideB = sideB;
        this.flowerMech = flowerMech;


    }

    public IntakeModule getModule(BeamBreakPosition.IntakeSide side) {
        return side == BeamBreakPosition.IntakeSide.A ? sideA : sideB;
    }

    /** The module facing a robot-frame forward (+X) direction or velocity: A for forward or zero, B for backward. */
    public IntakeModule getModuleFacing(double robotForward) {
        return robotForward >= 0 ? sideA : sideB;
    }

    public FlowerMech getFlower() {
        return flowerMech;
    }

    /** Open loop on both sides, the closest equivalent of last year's single intake motor. */
    public void run(double power) {
        sideA.run(power);
        sideB.run(power);
    }

    /** Closed loop on both sides at this motor RPS, each with its own gains. Call every loop; run() / stop() end it. */
    public void runRPS(double targetRPS) {
        sideA.runRPS(targetRPS);
        sideB.runRPS(targetRPS);
    }

    public void stop() {
        sideA.stop();
        sideB.stop();
    }

    /** Lowers both sides. Does not touch the flower. */
    public void lower() {
        sideA.lower();
        sideB.lower();
    }

    /** Raises both sides (the flower's side raises the flower first, see IntakeModule.raise). */
    public void raise() {
        sideA.raise();
        sideB.raise();
    }

    /** Call once per loop. */
    public void update() {
        sideA.update();
        sideB.update();
    }

    /** True when both sides are within toleranceRPS of targetRPS. */
    public boolean isAtRPS(double targetRPS, double toleranceRPS) {
        return sideA.isAtRPS(targetRPS, toleranceRPS) && sideB.isAtRPS(targetRPS, toleranceRPS);
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
