package com.kalipsorobotics.actions.actionUtilities;

import android.util.Log;

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
            Log.d("waitaction", "elapsed time " + elapsedTime.seconds());
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
