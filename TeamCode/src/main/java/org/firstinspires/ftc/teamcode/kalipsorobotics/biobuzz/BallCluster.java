package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * All balls inside one field tile, plus its center and score.
 * Coordinates are mm in the FieldConfig frame (garden corner is (0,0)).
 *
 * Usage:
 *   List<BallCluster> clusters = BallCluster.buildClusters(detectedBalls);
 *   BallCluster target = BallCluster.selectBest(clusters, robotPoint, allianceColor);
 *   if (target != null) drive to target.center
 */
public class BallCluster {
    public static final double TILE_SIZE_MM = 609.6;

    // Pollen = yellow, nectar = our alliance color
    public static final int POLLEN_POINTS = 1;
    public static final int NECTAR_POINTS = 2;

    // Ball diameter is 71 mm and x/y are ball centers, so detections of the same type
    // within half a diameter are treated as the same ball
    public static double DUPLICATE_TOLERANCE_MM = 35.5;

    // Max balls the intake can hold at once
    public static final int INTAKE_CAPACITY = 4;

    // TODO: rank by points per second once drive/turn speed matter, tune on the robot
    // public static double DRIVE_SPEED_MM_PER_SEC = 1000;
    // public static double INTAKE_TIME_PER_BALL_SEC = 0.5;

    public final int tileColumn;
    public final int tileRow;
    public final List<BallInformation> balls;
    public final Point center;
    private final EnumMap<BallInformation.Type, Integer> counts;

    private BallCluster(int tileColumn, int tileRow, List<BallInformation> balls) {
        this.tileColumn = tileColumn;
        this.tileRow = tileRow;
        this.balls = Collections.unmodifiableList(new ArrayList<>(balls));

        counts = new EnumMap<>(BallInformation.Type.class);
        for (BallInformation.Type type : BallInformation.Type.values()) {
            counts.put(type, 0);
        }

        double sumX = 0;
        double sumY = 0;
        for (BallInformation ball : balls) {
            sumX += ball.x;
            sumY += ball.y;
            counts.put(ball.type, counts.get(ball.type) + 1);
        }
        this.center = new Point(sumX / balls.size(), sumY / balls.size());
    }

    public int getCount(BallInformation.Type type) {
        return counts.get(type);
    }

    public int size() {
        return balls.size();
    }

    public boolean containsOpponent(AllianceColor alliance) {
        return getCount(opponentType(alliance)) > 0;
    }

    /** True if the cluster has an opponent or unknown ball and should never be collected. */
    public boolean isEliminated(AllianceColor alliance) {
        return containsOpponent(alliance) || getCount(BallInformation.Type.UNKNOWN) > 0;
    }

    /**
     * +2 per nectar, +1 per pollen, counting at most INTAKE_CAPACITY balls (nectar first).
     * Returns 0 if the cluster is eliminated.
     */
    public int getScore(AllianceColor alliance) {
        if (isEliminated(alliance)) {
            return 0;
        }
        int nectar = Math.min(getCount(allianceType(alliance)), INTAKE_CAPACITY);
        int pollen = Math.min(getCount(BallInformation.Type.YELLOW), INTAKE_CAPACITY - nectar);
        return nectar * NECTAR_POINTS + pollen * POLLEN_POINTS;
    }

    // TODO: use for ranking once drive/turn speed matter
    // /** Estimated seconds to drive to the center and intake the balls. */
    // public double getEstimatedTimeSec(Point robotPosition) {
    //     double driveTime = Point.distance(robotPosition, center) / DRIVE_SPEED_MM_PER_SEC;
    //     return driveTime + Math.min(size(), INTAKE_CAPACITY) * INTAKE_TIME_PER_BALL_SEC;
    // }
    //
    // /** Points per second. Higher is better. 0 if the cluster should not be collected. */
    // public double getEfficiency(Point robotPosition, AllianceColor alliance) {
    //     double time = getEstimatedTimeSec(robotPosition);
    //     if (time <= 0) {
    //         return 0;
    //     }
    //     return getScore(alliance) / time;
    // }

    public static int tileColumnOf(double x) {
        return (int) Math.floor(x / TILE_SIZE_MM);
    }

    public static int tileRowOf(double y) {
        return (int) Math.floor(y / TILE_SIZE_MM);
    }

    /** Removes duplicate detections, then groups the balls by the tile they are in. */
    public static List<BallCluster> buildClusters(List<BallInformation> detectedBalls) {
        List<BallInformation> uniqueBalls = new ArrayList<>();
        for (BallInformation ball : detectedBalls) {
            if (ball == null) {
                continue;
            }
            boolean duplicate = false;
            for (BallInformation kept : uniqueBalls) {
                if (ball.isSameBall(kept, DUPLICATE_TOLERANCE_MM)) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) {
                uniqueBalls.add(ball);
            }
        }

        Map<Long, List<BallInformation>> ballsByTile = new LinkedHashMap<>();
        for (BallInformation ball : uniqueBalls) {
            long key = tileKey(tileColumnOf(ball.x), tileRowOf(ball.y));
            List<BallInformation> tileBalls = ballsByTile.get(key);
            if (tileBalls == null) {
                tileBalls = new ArrayList<>();
                ballsByTile.put(key, tileBalls);
            }
            tileBalls.add(ball);
        }

        List<BallCluster> clusters = new ArrayList<>();
        for (List<BallInformation> tileBalls : ballsByTile.values()) {
            BallInformation first = tileBalls.get(0);
            clusters.add(new BallCluster(tileColumnOf(first.x), tileRowOf(first.y), tileBalls));
        }
        return clusters;
    }

    /**
     * Picks the cluster with the highest score.
     * Eliminated clusters (opponent or unknown ball) and clusters scoring 0 are never picked.
     * Ties go to the closer cluster. Returns null if nothing is worth collecting.
     */
    public static BallCluster selectBest(List<BallCluster> clusters, Point robotPosition, AllianceColor alliance) {
        BallCluster best = null;
        int bestScore = 0;
        double bestDistance = Double.MAX_VALUE;

        for (BallCluster cluster : clusters) {
            int score = cluster.getScore(alliance);
            if (score <= 0) {
                continue;
            }
            double distance = Point.distance(robotPosition, cluster.center);
            if (score > bestScore || (score == bestScore && distance < bestDistance)) {
                best = cluster;
                bestScore = score;
                bestDistance = distance;
            }
        }
        return best;
    }

    private static long tileKey(int column, int row) {
        return ((long) column << 32) | (row & 0xFFFFFFFFL);
    }

    private static BallInformation.Type allianceType(AllianceColor alliance) {
        return alliance == AllianceColor.RED ? BallInformation.Type.RED : BallInformation.Type.BLUE;
    }

    private static BallInformation.Type opponentType(AllianceColor alliance) {
        return alliance == AllianceColor.RED ? BallInformation.Type.BLUE : BallInformation.Type.RED;
    }

    @Override
    public String toString() {
        return String.format(Locale.US, "Tile(%d, %d) center (%.0f, %.0f) yellow=%d red=%d blue=%d unknown=%d",
                tileColumn, tileRow, center.getX(), center.getY(),
                getCount(BallInformation.Type.YELLOW), getCount(BallInformation.Type.RED),
                getCount(BallInformation.Type.BLUE), getCount(BallInformation.Type.UNKNOWN));
    }
}
