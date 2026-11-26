package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig.OUT_OF_RANGE_THRESHOLD;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.IShooterPredictor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.ShooterInterpolationDataLookup;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * The double-sided intake: module A (front, +X) and module B (back), plus the Flower on whichever
 * side ModuleConfig.FLOWER_SIDE says. Both-side commands are here; getModule() hands out one side
 * for callers that run them separately. Which side runs when is decided by the caller, not here.
 */
public class Intake {

    private final IntakeModule intakeSideA;
    private final IntakeModule intakeSideB;
    private final FlowerMech flowerMech;

    private final IShooterPredictor predictor;


    public Intake(OpModeUtilities opModeUtilities) {
        intakeSideA = new IntakeModule(opModeUtilities, BeamBreakPosition.IntakeSide.A, IntakeConfig.SIDE_A);
        intakeSideB = new IntakeModule(opModeUtilities, BeamBreakPosition.IntakeSide.B, IntakeConfig.SIDE_B);
        flowerMech = new FlowerMech(opModeUtilities, getModule(ModuleConfig.FLOWER_SIDE));

        predictor = new ShooterInterpolationDataLookup();
    }

    /** Direct-injection constructor (unit tests, public so actions tests can build one). The flower must already be attached to one of the modules. */
    public Intake(IntakeModule intakeSideA, IntakeModule intakeSideB, FlowerMech flowerMech) {
        this.intakeSideA = intakeSideA;
        this.intakeSideB = intakeSideB;
        this.flowerMech = flowerMech;
        this.predictor = null;
    }

    public IntakeModule getModule(BeamBreakPosition.IntakeSide side) {
        return side == BeamBreakPosition.IntakeSide.A ? intakeSideA : intakeSideB;
    }

    /** The module facing a robot-frame forward (+X) direction or velocity: A for forward or zero, B for backward. */
    public IntakeModule getModuleFacing(double robotForward) {
        return robotForward >= 0 ? intakeSideA : intakeSideB;
    }

    public FlowerMech getFlower() {
        return flowerMech;
    }

    /** Open loop on both sides, the closest equivalent of last year's single intake motor. */
    public void run(double power) {
        intakeSideA.run(power);
        intakeSideB.run(power);
    }

    /** Closed loop on both sides at this motor RPS, each with its own gains. Call every loop; run() / stop() end it. */
    public void runRPS(double targetRPS) {
        intakeSideA.runRPS(targetRPS);
        intakeSideB.runRPS(targetRPS);
    }

    public void stop() {
        intakeSideA.stop();
        intakeSideB.stop();
    }

    /** Lowers both sides. Does not touch the flower. */
    public void lower() {
        intakeSideA.lower();
        intakeSideB.lower();
    }

    /** Raises both sides (the flower's side raises the flower first, see IntakeModule.raise). */
    public void raise() {
        intakeSideA.raise();
        intakeSideB.raise();
    }

    /** Call once per loop. */
    public void update() {
        intakeSideA.update();
        intakeSideB.update();
    }

    /** True when both sides are within toleranceRPS of targetRPS. */
    public boolean isAtRPS(double targetRPS, double toleranceRPS) {
        return intakeSideA.isAtRPS(targetRPS, toleranceRPS) && intakeSideB.isAtRPS(targetRPS, toleranceRPS);
    }

    /** Balls held, as last published by BeamBreakArray to SharedData. */
    public int getBallCount() {
        return SharedData.getBallCount();
    }

    /** True when the published beam-break count is at capacity. */
    public boolean isFull() {
        return SharedData.getBeamBreakData().isFull();
    }

    public double getIntakeRPSPrediction(double distanceMM) {
        if (distanceMM > OUT_OF_RANGE_THRESHOLD) {
            if (KLog.isDebug()) {
                KLog.d("Intake", () -> "event=outOfRangeFallback distanceMm=" + distanceMM +
                        " fallbackMm=" + ShooterInterpolationConfig.FALLBACK_OUT_OF_RANGE_DISTANCE_MM);
            }
            return predictor.predict(ShooterInterpolationConfig.FALLBACK_OUT_OF_RANGE_DISTANCE_MM).intakeRPS;
        }
        return predictor.predict(distanceMM).intakeRPS;
    }

    public double getRPSSideA() {
        return intakeSideA.getRPS();
    }
    public double getRPSSideB() {
        return intakeSideB.getRPS();
    }

}
