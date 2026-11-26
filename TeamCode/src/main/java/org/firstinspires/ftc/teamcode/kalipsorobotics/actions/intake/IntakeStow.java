package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

/**
 * Stops both intakes, raises the Flower arm, then raises side A and side B (starting size, e.g. before parking).
 * Auto: {@code new IntakeStow(intake)}
 * Pattern source: PushBall (a KActionSet chain).
 */
public class IntakeStow extends KActionSet {
    public static final String TAG = "Intaking_Stow";

    public IntakeStow(Intake intake) {
        KLog.d(TAG, () -> "event=create");

        IntakeStop stop = new IntakeStop(intake);
        stop.setName("stowStop");
        this.addAction(stop);

        FlowerRaise flowerRaise = new FlowerRaise(intake.getFlower());
        flowerRaise.setName("stowFlowerRaise");
        flowerRaise.setDependentActions(stop);
        this.addAction(flowerRaise);

        IntakeModuleRaise raiseA = new IntakeModuleRaise(intake.getModule(BeamBreakPosition.IntakeSide.A));
        raiseA.setName("stowRaiseA");
        raiseA.setDependentActions(flowerRaise);
        this.addAction(raiseA);

        IntakeModuleRaise raiseB = new IntakeModuleRaise(intake.getModule(BeamBreakPosition.IntakeSide.B));
        raiseB.setName("stowRaiseB");
        raiseB.setDependentActions(flowerRaise);
        this.addAction(raiseB);
    }
}
