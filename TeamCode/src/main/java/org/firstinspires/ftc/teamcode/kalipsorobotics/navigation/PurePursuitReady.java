package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import java.util.List;

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
            final int resolvedIdx = pointIndex;
            List<Position> pts = purePursuitAction.getPathPoints();
            if (resolvedIdx >= 0 && resolvedIdx < pts.size()) {
                Position wp = pts.get(resolvedIdx);
                KLog.d("PurePursuitReady", () -> String.format(
                        "[%s] Watching pathPoints[%d]=(%.0f,%.0f) threshold=%.0fmm pp=%s",
                        getName(), resolvedIdx, wp.getX(), wp.getY(),
                        distanceThresholdMM, purePursuitAction.getName()));
            } else {
                KLog.d("PurePursuitReady", () -> String.format(
                        "[%s] WARNING: pointIndex=%d but pathPoints.size=%d pp=%s — path not built yet?",
                        getName(), resolvedIdx, pts.size(), purePursuitAction.getName()));
            }
        }
        boolean withinRange = purePursuitAction.isWithinDistancePoint(pointIndex, distanceThresholdMM);

        boolean ppDone = purePursuitAction.getIsDone();

        KLog.d("PurePursuitReady", () -> String.format("[%s] checking: isWithinRange=%b, ppDone=%b, ppName=%s",
                getName(), withinRange, ppDone, purePursuitAction.getName()));

        if (withinRange) {
            isDone = true;
            final int idx = pointIndex;
            List<Position> pts = purePursuitAction.getPathPoints();
            String ptStr = (idx >= 0 && idx < pts.size())
                    ? String.format("(%.0f,%.0f)", pts.get(idx).getX(), pts.get(idx).getY()) : "?";
            KLog.d("PurePursuitReady", () -> String.format(
                    "[%s] DONE — within %.0fmm of pathPoints[%d]=%s pp=%s",
                    getName(), distanceThresholdMM, idx, ptStr, purePursuitAction.getName()));
            return;
        }

        // Fallback: if PurePursuitAction is done but isWithinRange wasn't set, still mark as done
        if (ppDone) {
            final int idx = pointIndex;
            List<Position> pts = purePursuitAction.getPathPoints();
            String ptStr = (idx >= 0 && idx < pts.size())
                    ? String.format("(%.0f,%.0f)", pts.get(idx).getX(), pts.get(idx).getY()) : "?";
            KLog.d("PurePursuitReady", () -> String.format(
                    "[%s] DONE (fallback) — pp finished but robot never reached pathPoints[%d]=%s within %.0fmm pp=%s",
                    getName(), idx, ptStr, distanceThresholdMM, purePursuitAction.getName()));
            isDone = true;
        }
    }

    public void setDistanceThresholdMM(double distanceThresholdMM) {
        this.distanceThresholdMM = distanceThresholdMM;
    }
}
