package com.kalipsorobotics.actions.actionUtilities;

import com.kalipsorobotics.utilities.KLog;

import com.qualcomm.robotcore.util.ElapsedTime;

public class WaitAction extends Action {

    double waitTimeMS;
    ElapsedTime elapsedTime;

    public WaitAction(double targetWaitTimeMS) {
        this.dependentActions.add(new DoneStateAction());
        this.waitTimeMS = targetWaitTimeMS;
    }

    @Override
    protected boolean updateIsDone() {
        if (hasStarted) {
//            boolean done = elapsedTime.seconds() >= waitTimeSeconds;
            KLog.d("waitaction", "elapsed time " + elapsedTime.seconds());
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
