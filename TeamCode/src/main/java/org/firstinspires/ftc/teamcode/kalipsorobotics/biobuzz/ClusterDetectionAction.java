package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ClusterDetectionAction extends Action {

    public enum State {
        PROCESS_DETECTIONS,
        EVALUATE_BEST,
        DONE
    }

    private final List<BallInformation> rawDetections;
    private final Point robotPosition;
    private final AllianceColor allianceColor;

    private State currentState = State.PROCESS_DETECTIONS;
    private int rawIndex = 0;
    private int dedupIndex = 0;
    private int evalIndex = 0;

    // Working state
    private final List<BallInformation> uniqueBalls = new ArrayList<>();
    private final Map<Long, BallCluster> clustersByTile = new HashMap<>();
    private final List<BallCluster> clusterList = new ArrayList<>();

    // Results
    private BallCluster bestCluster = null;
    private int bestScore = 0;
    private double bestDistance = Double.MAX_VALUE;

    public ClusterDetectionAction(List<BallInformation> rawDetections, Point robotPosition, AllianceColor allianceColor) {
        this.rawDetections = rawDetections != null ? rawDetections : new ArrayList<>();
        this.robotPosition = robotPosition;
        this.allianceColor = allianceColor;
    }

    @Override
    protected void update() {
        switch (currentState) {

            // One duplicate comparison (or one accept) per update() call
            case PROCESS_DETECTIONS:
                if (rawIndex >= rawDetections.size()) {
                    currentState = State.EVALUATE_BEST;
                    return;
                }

                BallInformation candidate = rawDetections.get(rawIndex);

                if (candidate == null) {
                    rawIndex++;
                    dedupIndex = 0;
                    return;
                }

                // Compare against the next already accepted unique ball
                if (dedupIndex < uniqueBalls.size()) {
                    if (candidate.isSameBall(uniqueBalls.get(dedupIndex), BallCluster.DUPLICATE_TOLERANCE_MM)) {
                        rawIndex++;
                        dedupIndex = 0;
                        return;
                    }
                    dedupIndex++;
                    return;
                }

                // Checked against every unique ball with no match, so accept it
                uniqueBalls.add(candidate);

                int col = BallCluster.tileColumnOf(candidate.x);
                int row = BallCluster.tileRowOf(candidate.y);
                long key = tileKey(col, row);

                BallCluster cluster = clustersByTile.get(key);
                if (cluster == null) {
                    cluster = new BallCluster(col, row);
                    clustersByTile.put(key, cluster);
                    clusterList.add(cluster);
                }
                cluster.addBall(candidate);

                rawIndex++;
                dedupIndex = 0;
                break;

            // Evaluate ONE cluster per update() call
            case EVALUATE_BEST:
                if (evalIndex >= clusterList.size()) {
                    currentState = State.DONE;
                    isDone = true;
                    return;
                }

                BallCluster candidateCluster = clusterList.get(evalIndex);
                int score = candidateCluster.getScore(allianceColor);

                if (score > 0) {
                    double dist = Point.distance(robotPosition, candidateCluster.getCenter());
                    if (score > bestScore || (score == bestScore && dist < bestDistance)) {
                        bestCluster = candidateCluster;
                        bestScore = score;
                        bestDistance = dist;
                    }
                }
                evalIndex++;
                break;

            case DONE:
                break;
        }
    }

    public BallCluster getBestCluster() {
        return bestCluster;
    }

    public boolean isFinished() {
        return isDone;
    }

    private static long tileKey(int column, int row) {
        return ((long) column << 32) | (row & 0xFFFFFFFFL);
    }
}
