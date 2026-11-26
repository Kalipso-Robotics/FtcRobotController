package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

/**
 * Raises one intake side. If its Flower is down, the side raises the Flower first (interlock), so this keeps calling
 * side.update() until the side is up and IntakeConfig.PIVOT_MOVE_MS has passed.
 * Auto: {@code new IntakeModuleRaise(intake.getModule(BeamBreakPosition.IntakeSide.A))}
 * Pattern source: RunIntakeTime (timer) and CloseStopperAction (one servo move).
 */
public class IntakeModuleRaise extends Action {
    public static final String TAG = "Intaking_SideRaise";

    private final IntakeModule side;
    private final ElapsedTime timer = new ElapsedTime();

    public IntakeModuleRaise(IntakeModule side) {
        this.side = side;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        if (!hasStarted) {
            hasStarted = true;
            timer.reset();
            side.raise();
            KLog.d(TAG, () -> "name=" + getName() + " side=" + side.getSide() + " event=start");
        }
        side.update(); // finishes a raise that waited for the Flower
        if (side.isUp() && timer.milliseconds() >= IntakeConfig.PIVOT_MOVE_MS) {
            isDone = true;
            KLog.d(TAG, () -> "name=" + getName() + " side=" + side.getSide() + " event=done elapsedMs=" + timer.milliseconds());
        }
    }
}
