package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import java.util.List;

public class PurePursuitReady extends Action {
    private final IPurePursuitAction purePursuitAction;
    private final boolean trackLastPoint;
    private int pointIndex;
    private double distanceThresholdMM;

    public PurePursuitReady(IPurePursuitAction purePursuitAction, int pointIndex, double distanceThresholdMM) {
        this.purePursuitAction = purePursuitAction;
        this.pointIndex = pointIndex;
        this.distanceThresholdMM = distanceThresholdMM;
        this.trackLastPoint = (pointIndex < 0);
    }

    public PurePursuitReady(IPurePursuitAction purePursuitAction, double distanceThresholdMM) {
        //Cannot do pointIndex = lastIndex in construction time because points get added later.
        this(purePursuitAction, -1, distanceThresholdMM);
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }

        // In track-last-point mode, re-resolve every update so we follow dynamic
        // paths (e.g., vision-driven path replacements that change the last index).
        if (trackLastPoint) {
            int newIndex = purePursuitAction.getLastPointIndex();
            if (newIndex != pointIndex) {
                final int prev = pointIndex;
                final int next = newIndex;
                List<Position> pts = purePursuitAction.getPathPoints();
                String wpStr = (next >= 0 && next < pts.size())
                        ? String.format("(%.0f,%.0f)", pts.get(next).getX(), pts.get(next).getY())
                        : "?";
                KLog.d("PurePursuitReady", () -> String.format(
                        "[%s] last point moved %d → %d (now %s) pp=%s",
                        getName(), prev, next, wpStr, purePursuitAction.getName()));
                pointIndex = newIndex;
            }
        }

        boolean withinRange = pointIndex >= 0
                && purePursuitAction.isWithinDistancePoint(pointIndex, distanceThresholdMM);

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
