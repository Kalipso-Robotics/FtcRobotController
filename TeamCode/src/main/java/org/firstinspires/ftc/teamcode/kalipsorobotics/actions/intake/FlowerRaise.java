package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

/**
 * Raises the Flower arm (always allowed), done after IntakeConfig.FLOWER_MOVE_MS. The mirror of FlowerLower.
 * Auto: {@code new FlowerRaise(intake.getFlower())}
 * Pattern source: RunIntakeTime (timer) and CloseStopperAction (one servo move).
 */
public class FlowerRaise extends Action {
    public static final String TAG = "Intaking_Flower";

    private final FlowerMech flowerMech;
    private final ElapsedTime timer = new ElapsedTime();

    public FlowerRaise(FlowerMech flowerMech) {
        this.flowerMech = flowerMech;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        if (!hasStarted) {
            hasStarted = true;
            timer.reset();
            flowerMech.raise();
            KLog.d(TAG, () -> "name=" + getName() + " event=start raise");
        }
        if (timer.milliseconds() >= IntakeConfig.FLOWER_MOVE_MS) {
            isDone = true;
            KLog.d(TAG, () -> "name=" + getName() + " event=done raise elapsedMs=" + timer.milliseconds());
        }
    }
}
