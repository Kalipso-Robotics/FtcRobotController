package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

/**
 * Lowers one intake side, done after IntakeConfig.PIVOT_MOVE_MS.
 * Auto: {@code new IntakeModuleLower(intake.getModule(BeamBreakPosition.IntakeSide.A))}
 * Pattern source: RunIntakeTime (timer) and CloseStopperAction (one servo move).
 */
public class IntakeModuleLower extends Action {
    public static final String TAG = "Intaking_SideLower";

    private final IntakeModule side;
    private final ElapsedTime timer = new ElapsedTime();

    public IntakeModuleLower(IntakeModule side) {
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
            side.lower();
            KLog.d(TAG, () -> "name=" + getName() + " side=" + side.getSide() + " event=start");
        }
        if (timer.milliseconds() >= IntakeConfig.PIVOT_MOVE_MS) {
            isDone = true;
            KLog.d(TAG, () -> "name=" + getName() + " side=" + side.getSide() + " event=done elapsedMs=" + timer.milliseconds());
        }
    }
}
