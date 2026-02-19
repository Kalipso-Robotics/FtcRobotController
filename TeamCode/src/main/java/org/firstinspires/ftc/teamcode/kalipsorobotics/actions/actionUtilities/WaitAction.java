package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import com.qualcomm.robotcore.util.ElapsedTime;

public class WaitAction extends Action {

    double waitTimeMS;
    ElapsedTime elapsedTime;

    public WaitAction(double targetWaitTimeMS) {
        this.dependentActions.add(new DoneStateAction());
        this.waitTimeMS = targetWaitTimeMS;
    }

    @Override
    protected boolean isUpdateDone() {
        if (hasStarted) {
//            boolean done = elapsedTime.seconds() >= waitTimeSeconds;
            KLog.d("waitaction", () -> "elapsed time " + elapsedTime.seconds());
            isDone = elapsedTime.milliseconds() >= waitTimeMS;
            return isDone;
        }

        return false;
    }

    @Override
    public void update() {
        if(!hasStarted) {
            elapsedTime = new ElapsedTime();
            hasStarted = true;
        }
    }
}
