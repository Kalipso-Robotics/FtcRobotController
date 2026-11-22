package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

/**
 * Runs whichever intake side faces the way the robot drives (DriveDirectionIntakeController) every loop. It never
 * finishes by itself: call setIsDone(true) (or drop it), which stops both sides and raises them.
 * Auto: run it next to a drive, e.g. {@code KActionSet{drive, new DriveDirectionIntakeAction(intake)}}, and setIsDone when the drive ends.
 * Pattern source: RunIntakeTime (per-loop intake action).
 */
public class DriveDirectionIntakeAction extends Action {
    public static final String TAG = "Intaking_DriveDirection";

    private final Intake intake;
    private final DriveDirectionIntakeController controller;

    public DriveDirectionIntakeAction(Intake intake) {
        this.intake = intake;
        this.controller = new DriveDirectionIntakeController(intake);
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        if (!hasStarted) {
            hasStarted = true;
            KLog.d(TAG, () -> "name=" + getName() + " event=start");
        }
        controller.update();
    }

    @Override
    public void setIsDone(boolean isDone) {
        boolean wasDone = this.isDone;
        super.setIsDone(isDone);
        if (!isDone || wasDone) {
            return;
        }
        for (BeamBreakPosition.IntakeSide s : BeamBreakPosition.IntakeSide.values()) {
            intake.getSide(s).stop();
            intake.getSide(s).raise();
        }
        KLog.d(TAG, () -> "name=" + getName() + " event=done");
    }
}
