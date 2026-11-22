package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

/**
 * Lowers the Flower arm. The arm may only go down while its intake side is down, so this retries lower() every loop
 * until the interlock allows it, then is done after IntakeConfig.FLOWER_MOVE_MS. If it is still refused after
 * timeoutMS it logs KLog.e Intaking_Flower event=lowerTimeout and finishes anyway (the auto keeps going).
 * Auto: {@code new FlowerLower(intake.getFlower(), 1000)} after an IntakeSideLower on the flower's side.
 * Pattern source: RunIntakeTime (timer) and CloseStopperAction (one servo move).
 */
public class FlowerLower extends Action {
    public static final String TAG = "Intaking_Flower";

    private final FlowerMech flowerMech;
    private final double timeoutMS;
    private final ElapsedTime timer = new ElapsedTime();
    private ElapsedTime movingTimer = null;

    public FlowerLower(FlowerMech flowerMech, double timeoutMS) {
        this.flowerMech = flowerMech;
        this.timeoutMS = timeoutMS;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        if (!hasStarted) {
            hasStarted = true;
            timer.reset();
            KLog.d(TAG, () -> "name=" + getName() + " event=start");
        }
        if (movingTimer == null) {
            if (flowerMech.lower()) {
                movingTimer = new ElapsedTime();
            } else if (timer.milliseconds() >= timeoutMS) {
                isDone = true;
                KLog.e(TAG, "name=" + getName() + " event=lowerTimeout timeoutMs=" + timeoutMS);
                return;
            }
        }
        if (movingTimer != null && movingTimer.milliseconds() >= IntakeConfig.FLOWER_MOVE_MS) {
            isDone = true;
            KLog.d(TAG, () -> "name=" + getName() + " event=done elapsedMs=" + timer.milliseconds());
        }
    }
}
