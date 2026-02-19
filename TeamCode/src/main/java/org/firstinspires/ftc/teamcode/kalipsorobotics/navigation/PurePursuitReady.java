package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

public class PurePursuitReady extends Action {
    private final PurePursuitAction purePursuitAction;
    private int pointIndex;
    private double distanceThresholdMM;

    public PurePursuitReady(PurePursuitAction purePursuitAction, int pointIndex, double distanceThresholdMM) {
        this.purePursuitAction = purePursuitAction;
        this.pointIndex = pointIndex;
        this.distanceThresholdMM = distanceThresholdMM;
    }

    public PurePursuitReady(PurePursuitAction purePursuitAction, double distanceThresholdMM) {
        //Cannot do pointIndex = lastIndex in construction time because points get added later.
        this(purePursuitAction, -1, distanceThresholdMM);
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }

        if (pointIndex < 0) {
            pointIndex = purePursuitAction.getLastPointIndex();
        }
        boolean withinRange = purePursuitAction.isWithinDistancePoint(pointIndex, distanceThresholdMM);

        boolean ppDone = purePursuitAction.getIsDone();

        KLog.d("PurePursuitReady", () -> String.format("[%s] checking: isWithinRange=%b, ppDone=%b, ppName=%s",
                getName(), withinRange, ppDone, purePursuitAction.getName()));

        if (withinRange) {
            isDone = true;
            KLog.d("PurePursuitReady", () -> "PurePursuit IS WITHIN RANGE, SETTING DONE TO READY: " + purePursuitAction.getName());
            return;
        }

        // Fallback: if PurePursuitAction is done but isWithinRange wasn't set, still mark as done
        if (ppDone) {
            KLog.d("PurePursuitReady", () -> String.format("[%s] WARNING: PP is done but isWithinRange=false! Marking ready anyway.", getName()));
            isDone = true;
        }
    }

    public void setDistanceThresholdMM(double distanceThresholdMM) {
        this.distanceThresholdMM = distanceThresholdMM;
    }
}
