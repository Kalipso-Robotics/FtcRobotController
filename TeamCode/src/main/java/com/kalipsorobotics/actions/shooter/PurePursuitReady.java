package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.utilities.KLog;

public class PurePursuitReady extends Action {
    private PurePursuitAction purePursuitAction;
    public PurePursuitReady(PurePursuitAction purePursuitAction) {
        this.purePursuitAction = purePursuitAction;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }

        boolean withinRange = purePursuitAction.isWithinRange();
        boolean ppDone = purePursuitAction.getIsDone();

        KLog.d("PurePursuitReady", String.format("[%s] checking: isWithinRange=%b, ppDone=%b, ppName=%s",
                getName(), withinRange, ppDone, purePursuitAction.getName()));

        if (withinRange) {
            isDone = true;
            KLog.d("PurePursuitReady", "PurePursuit IS WITHIN RANGE, SETTING DONE TO READY: " + purePursuitAction.getName());
            return;
        }

        // Fallback: if PurePursuitAction is done but isWithinRange wasn't set, still mark as done
        if (ppDone) {
            KLog.d("PurePursuitReady", String.format("[%s] WARNING: PP is done but isWithinRange=false! Marking ready anyway.", getName()));
            isDone = true;
        }
    }
}
